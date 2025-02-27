// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
// Description:
// This is the sequencer of one lane. It controls the execution of one vector
// instruction within one lane, interfacing with the internal functional units
// and with the main sequencer.

// 是顺序发射的
// 不能背靠背发射同一功能单元的指令，两条使用不同功能单元的指令可以背靠背发射，否则要延1拍以上
// 指令间的冒险operand requester处理调度，sequencer只负责找出指令间的冒险
// 负责指令的内部发射，接受到一条新指令且与main sequencer握手成功，会在下一周期向operand requester发送操作数请求，operand requester接受到请求就会向操作数队列申请一项空间
// 同时握手成功在下个时钟沿也会向vfu发送一条新指令，vfu接受到指令信息后就会在指令队列中分配一项空间，无需和vfu握手
// lane sequencer不负责功能单元的空满判断，只负责发射，lane是否满了由main sequcner判断
module lane_sequencer import ara_pkg::*; import rvv_pkg::*; import cf_math_pkg::idx_width; #(
    parameter int unsigned NrLanes = 0
  ) (
    input  logic                                          clk_i,
    input  logic                                          rst_ni,
    // Lane ID
    input  logic                 [idx_width(NrLanes)-1:0] lane_id_i,
    // Interface with the main sequencer
    input  pe_req_t                                       pe_req_i, // 主定序器发送过来的指令请求，包括向量寄存器编号，标量寄存器的值，指令id等等
    input  logic                                          pe_req_valid_i,
    input  logic                 [NrVInsn-1:0]            pe_vinsn_running_i, // 指令运行状态，每一位代表指令是否在运行
    output logic                                          pe_req_ready_o,
    output pe_resp_t                                      pe_resp_o, // lane sequencer会设置其中的vinsn_done，代表指令执行完的情况，每一位代表该指令是否执行完，这是一个向量
    // Interface with the operand requester
    output operand_request_cmd_t [NrOperandQueues-1:0]    operand_request_o, // 发射出的指令的操作数信息，会分割给不同的功能单元
    output logic                 [NrOperandQueues-1:0]    operand_request_valid_o,
    input  logic                 [NrOperandQueues-1:0]    operand_request_ready_i,
    output logic                                          alu_vinsn_done_o, // lane当作功能单元来处理，当alu ready时，该信号需要打一拍后才能传给主定序器
    output logic                                          mfpu_vinsn_done_o,
    // Interface with the lane's VFUs
    output vfu_operation_t                                vfu_operation_o,
    output logic                                          vfu_operation_valid_o,
    input  logic                                          alu_ready_i,
    input  logic                 [NrVInsn-1:0]            alu_vinsn_done_i,
    input  logic                                          mfpu_ready_i,
    input  logic                 [NrVInsn-1:0]            mfpu_vinsn_done_i
  );

  ////////////////////////////
  //  Register the request  //
  ////////////////////////////

  // Don't accept the same request more than once!
  // The main sequencer keeps the valid high and broadcast
  // a certain instruction with ID == X to all the lanes
  // until every lane has sampled it.

  // Every time a lane handshakes the main sequencer, it also
  // saves the insn ID, not to re-sample the same instruction.
  vid_t last_id_d, last_id_q;
  logic pe_req_valid_i_msk;
  logic en_sync_mask_d, en_sync_mask_q;

  pe_req_t pe_req;
  logic    pe_req_valid;
  logic    pe_req_ready;

  // 功能描述：1项的fifo，当fifo为空且输入输出有效，则直接将输入的值转发到输出端口，否则入队
  // 当发现重复id时不入队，由信号pe_req_valid_i_msk控制
  fall_through_register #(
    .T(pe_req_t)
  ) i_pe_req_register (
    .clk_i     (clk_i             ),
    .rst_ni    (rst_ni            ),
    .clr_i     (1'b0              ),
    .testmode_i(1'b0              ),
    .data_i    (pe_req_i          ),
    .valid_i   (pe_req_valid_i_msk),
    .ready_o   (pe_req_ready_o    ),
    .data_o    (pe_req            ),
    .valid_o   (pe_req_valid      ),
    .ready_i   (pe_req_ready      )
  );

  // 当fall_through_register与主定序器握手成功时，会进行：锁存指令id last_id_q、设置同步位en_sync_mask_q，锁存主定序器请求pe_req
  // 当同步位设置时，每次都会比较last_id和新来的指令的id，避免重复采样，由pe_req_valid_i_msk信号来控制id、同步位和pe_req的锁存
  // 该信号默认为pe_req_valid_i，即主定序器发来的握手请求
  // 当没有请求发来时，同步位会设为0
  always_comb begin
    // Default assignment
    last_id_d      = last_id_q;
    en_sync_mask_d = en_sync_mask_q;

    // If the sync mask is enabled and the ID is the same
    // as before, avoid to re-sample the same instruction
    // more than once.
    if (en_sync_mask_q && (pe_req_i.id == last_id_q))
      pe_req_valid_i_msk = 1'b0;
    else
      pe_req_valid_i_msk = pe_req_valid_i;

    // Enable the sync mask when a handshake happens,
    // and save the insn ID
    if (pe_req_valid_i_msk && pe_req_ready_o) begin
      last_id_d      = pe_req_i.id;
      en_sync_mask_d = 1'b1;
    end

    // Disable the block if the sequencer valid goes down
    if (!pe_req_valid_i && en_sync_mask_q)
      en_sync_mask_d = 1'b0;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      last_id_q      <= '0;
      en_sync_mask_q <= 1'b0;
    end else begin
      last_id_q      <= last_id_d;
      en_sync_mask_q <= en_sync_mask_d;
    end
  end

  //////////////////////////////////////
  //  Operand Request Command Queues  //
  //////////////////////////////////////

  // We cannot use a simple FIFO because the operand request commands include
  // bits that indicate whether there is a hazard between different vector
  // instructions. Such hazards must be continuously cleared based on the
  // value of the currently running loops from the main sequencer.
  operand_request_cmd_t [NrOperandQueues-1:0] operand_request_i;
  logic                 [NrOperandQueues-1:0] operand_request_push;

  operand_request_cmd_t [NrOperandQueues-1:0] operand_request_d;
  logic                 [NrOperandQueues-1:0] operand_request_valid_d;

  // 当push[i]为1，则将operand_request_i[i]放入对应部分，当operand requester中发过来了某一项的ready，则清空该项中对应部分
  // 这只是fifo中的一项，fifo中的一项分割成了9个部分，包括AluA, AluB, MulFPUA, MulFPUB, MulFPUC, MaskB, MaskM, StA, SlideAddrGenA
  always_comb begin: p_operand_request
    for (int queue = 0; queue < NrOperandQueues; queue++) begin
      // Maintain state
      operand_request_d[queue]       = operand_request_o[queue];
      operand_request_valid_d[queue] = operand_request_valid_o[queue];

      // Clear the request
      if (operand_request_ready_i[queue]) begin
        operand_request_d[queue]       = '0;
        operand_request_valid_d[queue] = 1'b0;
      end

      // Got a new request
      if (operand_request_push[queue]) begin
        operand_request_d[queue]       = operand_request_i[queue];
        operand_request_valid_d[queue] = 1'b1;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin: p_operand_request_ff
    if (!rst_ni) begin
      operand_request_o       <= '0;
      operand_request_valid_o <= '0;
    end else begin
      operand_request_o       <= operand_request_d;
      operand_request_valid_o <= operand_request_valid_d;
    end
  end

  /////////////////////////////
  //  VFU Operation control  //
  /////////////////////////////

  // Running instructions
  logic [NrVInsn-1:0] vinsn_done_d, vinsn_done_q;
  logic [NrVInsn-1:0] vinsn_running_d, vinsn_running_q;

  // VFU operation
  vfu_operation_t vfu_operation_d;
  logic           vfu_operation_valid_d;

  // Cut the path
  logic alu_vinsn_done_d, mfpu_vinsn_done_d;

  // Returns true if the corresponding lane VFU is ready.
  function automatic logic vfu_ready(vfu_e vfu, logic alu_ready_i, logic mfpu_ready_i);
    vfu_ready = 1'b1;
    unique case (vfu)
      VFU_Alu,
      VFU_MaskUnit: vfu_ready = alu_ready_i;
      VFU_MFpu    : vfu_ready = mfpu_ready_i;
      default:;
    endcase
  endfunction : vfu_ready

  // 根据发送来的指令功能单元类型，如果fifo entry中对应功能单元的部分是空闲的，则会产生pe_req_ready，提示可以接受下一条指令
  // 如果lane_sequencer中的fifo entry和fall_through_register握手成功且该指令没有在运行，则计算通道需要计算的次数和操作数相关的信号，并在下一个周期将指令标记为运行
  always_comb begin: sequencer
    // Running loops
    vinsn_running_d = vinsn_running_q & pe_vinsn_running_i;

    // Ready to accept a new request, by default
    pe_req_ready = 1'b1;

    // Loops that finished execution
    vinsn_done_d         = alu_vinsn_done_i | mfpu_vinsn_done_i;
    alu_vinsn_done_d     = |alu_vinsn_done_i;
    mfpu_vinsn_done_d    = |mfpu_vinsn_done_i;
    pe_resp_o.vinsn_done = vinsn_done_q;

    // Make no requests to the operand requester
    operand_request_i    = '0;
    operand_request_push = '0;

    // Make no requests to the lane's VFUs
    vfu_operation_d       = '0;
    vfu_operation_valid_d = 1'b0;

    // If the operand requesters are busy, abort the request and wait for another cycle.
    // 此处不能背靠背执行一条指令？
    if (pe_req_valid) begin
      unique case (pe_req.vfu)
        VFU_Alu : begin
          pe_req_ready = !(operand_request_valid_o[AluA] ||
            operand_request_valid_o[AluB ] ||
            operand_request_valid_o[MaskM]);
        end
        VFU_MFpu : begin
          pe_req_ready = !(operand_request_valid_o[MulFPUA] ||
            operand_request_valid_o[MulFPUB] ||
            operand_request_valid_o[MulFPUC] ||
            operand_request_valid_o[MaskM]);
        end
        VFU_LoadUnit : pe_req_ready = !(operand_request_valid_o[MaskM] ||
            (pe_req_i.op == VLXE && operand_request_valid_o[SlideAddrGenA])); // 索引访存
        VFU_SlideUnit: pe_req_ready = !(operand_request_valid_o[SlideAddrGenA]);
        VFU_StoreUnit: begin
          pe_req_ready = !(operand_request_valid_o[StA] ||
            operand_request_valid_o[MaskM] ||
            (pe_req_i.op == VSXE && operand_request_valid_o[SlideAddrGenA]));
        end
        VFU_MaskUnit : begin
          pe_req_ready = !(operand_request_valid_o[AluA] ||
            operand_request_valid_o [AluB] ||
            operand_request_valid_o [MulFPUA] ||
            operand_request_valid_o [MulFPUB] ||
            operand_request_valid_o[MaskB] ||
            operand_request_valid_o[MaskM]);
        end
        VFU_None : begin
          pe_req_ready = !(operand_request_valid_o[MaskB]);
        end
        default:;
      endcase
    end

    // We received a new vector instruction
    // 握手成功而且该指令没有在运行，向vfu发送一条指令，向operand requester发送一条操作数请求
    if (pe_req_valid && pe_req_ready && !vinsn_running_d[pe_req.id]) begin
      // Populate the VFU request
      vfu_operation_d = '{
        id             : pe_req.id,
        op             : pe_req.op,
        vm             : pe_req.vm,
        vfu            : pe_req.vfu,
        use_vs1        : pe_req.use_vs1,
        use_vs2        : pe_req.use_vs2,
        use_vd_op      : pe_req.use_vd_op,
        scalar_op      : pe_req.scalar_op,
        use_scalar_op  : pe_req.use_scalar_op,
        vd             : pe_req.vd,
        use_vd         : pe_req.use_vd,
        swap_vs2_vd_op : pe_req.swap_vs2_vd_op,
        fp_rm          : pe_req.fp_rm,
        wide_fp_imm    : pe_req.wide_fp_imm,
        cvt_resize     : pe_req.cvt_resize,
        vtype          : pe_req.vtype,
        default        : '0
      };
      // 似乎不需要和vfu握手？main sequencer知晓指令队列的剩余空间，因此只要发射到了lane sequencer，就说明指令队列还有空间
      vfu_operation_valid_d = (vfu_operation_d.vfu != VFU_None) ? 1'b1 : 1'b0;

      // Vector length calculation
      // 该通道需要计算的向量元素个数，这里有个除法器？？？
      vfu_operation_d.vl = pe_req.vl / NrLanes;
      // If lane_id_i < vl % NrLanes, this lane has to execute one extra micro-operation.
      if (lane_id_i < pe_req.vl[idx_width(NrLanes)-1:0]) vfu_operation_d.vl += 1;

      // Vector start calculation
      vfu_operation_d.vstart = pe_req.vstart / NrLanes;
      // If lane_id_i < vstart % NrLanes, this lane needs to execute one micro-operation less.
      if (lane_id_i < pe_req.vstart[idx_width(NrLanes)-1:0]) vfu_operation_d.vstart -= 1;

      // Mark the vector instruction as running
      vinsn_running_d[pe_req.id] = (vfu_operation_d.vfu != VFU_None) ? 1'b1 : 1'b0;

      // 当该通道不需要计算且功能单元是alu或者fpu时，该通道的状态会设定为执行完毕，又可以继续接受其他指令，不会向operand requester发送新的请求
      // Mute request if the instruction runs in the lane and the vl is zero.
      // Exception 1: insn on mask vectors, as MASKU has to receive something from all lanes
      // and the partial results come from VALU and VMFPU.
      // Exception 2: during a reduction, all the lanes must cooperate anyway.
      if (vfu_operation_d.vl == '0 && (vfu_operation_d.vfu inside {VFU_Alu, VFU_MFpu}) && !(vfu_operation_d.op inside {[VREDSUM:VWREDSUM], [VFREDUSUM:VFWREDOSUM]})) begin
        vfu_operation_valid_d = 1'b0;
        // We are already done with this instruction
        vinsn_done_d[pe_req.id] |= 1'b1;
        vinsn_running_d[pe_req.id] = 1'b0;
      end

      ////////////////////////
      //  Operand requests  //
      ////////////////////////

      unique case (pe_req.vfu)
        VFU_Alu: begin
          operand_request_i[AluA] = '{
            id         : pe_req.id,
            vs         : pe_req.vs1,
            eew        : pe_req.eew_vs1,
            // If reductions and vl == 0, we must replace with neutral values
            conv       : (vfu_operation_d.vl == '0) ? OpQueueReductionZExt : pe_req.conversion_vs1,
            scale_vl   : pe_req.scale_vl,
            cvt_resize : pe_req.cvt_resize,
            vtype      : pe_req.vtype,
            // In case of reduction, AluA opqueue will keep the scalar element
            vl         : (pe_req.op inside {[VREDSUM:VWREDSUM]}) ? 1 : vfu_operation_d.vl,
            vstart     : vfu_operation_d.vstart,
            hazard     : pe_req.hazard_vs1 | pe_req.hazard_vd,
            is_reduct  : pe_req.op inside {[VREDSUM:VWREDSUM]} ? 1'b1 : 0,
            target_fu  : ALU_SLDU,
            default    : '0
          };
          operand_request_push[AluA] = pe_req.use_vs1;

          operand_request_i[AluB] = '{
            id         : pe_req.id,
            vs         : pe_req.vs2,
            eew        : pe_req.eew_vs2,
            // If reductions and vl == 0, we must replace with neutral values
            conv       : (vfu_operation_d.vl == '0) ? OpQueueReductionZExt : pe_req.conversion_vs2,
            scale_vl   : pe_req.scale_vl,
            cvt_resize : pe_req.cvt_resize,
            vtype      : pe_req.vtype,
            // If reductions and vl == 0, we must replace the operands with neutral
            // values in the opqueues. So, vl must be 1 at least
            vl         : (pe_req.op inside {[VREDSUM:VWREDSUM]} && vfu_operation_d.vl == '0)
                         ? 1 : vfu_operation_d.vl,
            vstart     : vfu_operation_d.vstart,
            hazard     : pe_req.hazard_vs2 | pe_req.hazard_vd,
            is_reduct  : pe_req.op inside {[VREDSUM:VWREDSUM]} ? 1'b1 : 0,
            target_fu  : ALU_SLDU,
            default    : '0
          };
          operand_request_push[AluB] = pe_req.use_vs2;

          // This vector instruction uses masks
          operand_request_i[MaskM] = '{
            id     : pe_req.id,
            vs     : VMASK,
            eew    : pe_req.vtype.vsew,
            vtype  : pe_req.vtype,
            // Since this request goes outside of the lane, we might need to request an
            // extra operand regardless of whether it is valid in this lane or not.
            vl     : (pe_req.vl / NrLanes / 8) >> int'(pe_req.vtype.vsew),
            vstart : vfu_operation_d.vstart,
            hazard : pe_req.hazard_vm | pe_req.hazard_vd,
            default: '0
          };
          if ((operand_request_i[MaskM].vl << int'(pe_req.vtype.vsew)) *
              NrLanes * 8 != pe_req.vl) operand_request_i[MaskM].vl += 1;
          operand_request_push[MaskM] = !pe_req.vm;
        end
        VFU_MFpu: begin
          operand_request_i[MulFPUA] = '{
            id         : pe_req.id,
            vs         : pe_req.vs1,
            eew        : pe_req.eew_vs1,
            // If reductions and vl == 0, we must replace with neutral values
            conv       : pe_req.conversion_vs1,
            scale_vl   : pe_req.scale_vl,
            cvt_resize : pe_req.cvt_resize,
            vtype      : pe_req.vtype,
            // If reductions and vl == 0, we must replace the operands with neutral
            // values in the opqueues. So, vl must be 1 at least
            vl         : (pe_req.op inside {[VFREDUSUM:VFWREDOSUM]}) ? 1 : vfu_operation_d.vl,
            vstart     : vfu_operation_d.vstart,
            hazard     : pe_req.hazard_vs1 | pe_req.hazard_vd,
            is_reduct  : pe_req.op inside {[VFREDUSUM:VFWREDOSUM]} ? 1'b1 : 0,
            target_fu  : MFPU_ADDRGEN,
            default    : '0
          };
          operand_request_push[MulFPUA] = pe_req.use_vs1;

          operand_request_i[MulFPUB] = '{
            id         : pe_req.id,
            vs         : pe_req.swap_vs2_vd_op ? pe_req.vd        : pe_req.vs2,
            eew        : pe_req.swap_vs2_vd_op ? pe_req.eew_vd_op : pe_req.eew_vs2,
            // If reductions and vl == 0, we must replace with neutral values
            conv       : pe_req.conversion_vs2,
            scale_vl   : pe_req.scale_vl,
            cvt_resize : pe_req.cvt_resize,
            vtype      : pe_req.vtype,
            // If reductions and vl == 0, we must replace the operands with neutral
            // values in the opqueues. So, vl must be 1 at least
            vl         : (pe_req.op inside {[VFREDUSUM:VFWREDOSUM]} && vfu_operation_d.vl == '0)
                        ? 1 : vfu_operation_d.vl,
            vstart     : vfu_operation_d.vstart,
            hazard     : (pe_req.swap_vs2_vd_op ?
            pe_req.hazard_vd : (pe_req.hazard_vs2 | pe_req.hazard_vd)),
            is_reduct  : pe_req.op inside {[VFREDUSUM:VFWREDOSUM]} ? 1'b1 : 0,
            target_fu  : MFPU_ADDRGEN,
            default: '0
          };
          operand_request_push[MulFPUB] = pe_req.swap_vs2_vd_op ?
          pe_req.use_vd_op : pe_req.use_vs2;

          operand_request_i[MulFPUC] = '{
            id         : pe_req.id,
            vs         : pe_req.swap_vs2_vd_op ? pe_req.vs2            : pe_req.vd,
            eew        : pe_req.swap_vs2_vd_op ? pe_req.eew_vs2        : pe_req.eew_vd_op,
            conv       : pe_req.swap_vs2_vd_op ? pe_req.conversion_vs2 : OpQueueConversionNone,
            scale_vl   : pe_req.scale_vl,
            cvt_resize : pe_req.cvt_resize,
            // If reductions and vl == 0, we must replace the operands with neutral
            // values in the opqueues. So, vl must be 1 at least
            vl         : (pe_req.op inside {[VFREDUSUM:VFWREDOSUM]} && vfu_operation_d.vl == '0)
                        ? 1 : vfu_operation_d.vl,
            vstart     : vfu_operation_d.vstart,
            vtype      : pe_req.vtype,
            hazard     : pe_req.swap_vs2_vd_op ?
            (pe_req.hazard_vs2 | pe_req.hazard_vd) : pe_req.hazard_vd,
            is_reduct  : pe_req.op inside {[VFREDUSUM:VFWREDOSUM]} ? 1'b1 : 0,
            target_fu  : MFPU_ADDRGEN,
            default : '0
          };
          operand_request_push[MulFPUC] = pe_req.swap_vs2_vd_op ?
          pe_req.use_vs2 : pe_req.use_vd_op;

          // This vector instruction uses masks
          operand_request_i[MaskM] = '{
            id     : pe_req.id,
            vs     : VMASK, // 掩码寄存器总是0号寄存器
            eew    : pe_req.vtype.vsew,
            vtype  : pe_req.vtype,
            // Since this request goes outside of the lane, we might need to request an
            // extra operand regardless of whether it is valid in this lane or not.
            vl     : (pe_req.vl / NrLanes / 8) >> int'(pe_req.vtype.vsew),
            vstart : vfu_operation_d.vstart,
            hazard : pe_req.hazard_vm | pe_req.hazard_vd,
            default: '0
          };
          if ((operand_request_i[MaskM].vl << int'(pe_req.vtype.vsew)) *
              NrLanes * 8 != pe_req.vl) operand_request_i[MaskM].vl += 1;
          operand_request_push[MaskM] = !pe_req.vm;
        end
        VFU_LoadUnit : begin
          // This vector instruction uses masks
          operand_request_i[MaskM] = '{
            id     : pe_req.id,
            vs     : VMASK,
            eew    : pe_req.vtype.vsew,
            vtype  : pe_req.vtype,
            // Since this request goes outside of the lane, we might need to request an
            // extra operand regardless of whether it is valid in this lane or not.
            vl     : (pe_req.vl / NrLanes / 8) >> int'(pe_req.vtype.vsew),
            vstart : vfu_operation_d.vstart,
            hazard : pe_req.hazard_vm | pe_req.hazard_vd,
            default: '0
          };
          if ((operand_request_i[MaskM].vl << int'(pe_req.vtype.vsew)) *
              NrLanes * 8 != pe_req.vl) operand_request_i[MaskM].vl += 1;
          operand_request_push[MaskM] = !pe_req.vm;

          // Load indexed
          operand_request_i[SlideAddrGenA] = '{
            id       : pe_req_i.id,
            vs       : pe_req_i.vs2,
            eew      : pe_req_i.eew_vs2,
            conv     : pe_req_i.conversion_vs2,
            target_fu: MFPU_ADDRGEN,
            vl       : pe_req_i.vl / NrLanes,
            scale_vl : pe_req_i.scale_vl,
            vstart   : vfu_operation_d.vstart,
            vtype    : pe_req_i.vtype,
            hazard   : pe_req_i.hazard_vs2 | pe_req_i.hazard_vd,
            default  : '0
          };
          // Since this request goes outside of the lane, we might need to request an
          // extra operand regardless of whether it is valid in this lane or not.
          if (operand_request_i[SlideAddrGenA].vl * NrLanes != pe_req_i.vl)
            operand_request_i[SlideAddrGenA].vl += 1;
          operand_request_push[SlideAddrGenA] = pe_req_i.op == VLXE;
        end

        VFU_StoreUnit : begin
          operand_request_i[StA] = '{
            id      : pe_req.id,
            vs      : pe_req.vs1,
            eew     : pe_req.eew_vs1,
            conv    : pe_req.conversion_vs1,
            scale_vl: pe_req.scale_vl,
            vtype   : pe_req.vtype,
            // Since this request goes outside of the lane, we might need to request an
            // extra operand regardless of whether it is valid in this lane or not.
            vl      : pe_req.vl / NrLanes,
            vstart  : vfu_operation_d.vstart,
            hazard  : pe_req.hazard_vs1 | pe_req.hazard_vd,
            default : '0
          };
          if (operand_request_i[StA].vl * NrLanes != pe_req.vl) operand_request_i[StA].vl += 1;
          operand_request_push[StA] = pe_req.use_vs1;

          // This vector instruction uses masks
          operand_request_i[MaskM] = '{
            id     : pe_req.id,
            vs     : VMASK,
            eew    : pe_req.vtype.vsew,
            vtype  : pe_req.vtype,
            // Since this request goes outside of the lane, we might need to request an
            // extra operand regardless of whether it is valid in this lane or not.
            vl     : (pe_req.vl / NrLanes / 8) >> int'(pe_req.vtype.vsew),
            vstart : vfu_operation_d.vstart,
            hazard : pe_req.hazard_vm | pe_req.hazard_vd,
            default: '0
          };
          if ((operand_request_i[MaskM].vl << int'(pe_req.vtype.vsew)) *
              NrLanes * 8 != pe_req.vl) operand_request_i[MaskM].vl += 1;
          operand_request_push[MaskM] = !pe_req.vm;

          // Store indexed
          operand_request_i[SlideAddrGenA] = '{
            id       : pe_req_i.id,
            vs       : pe_req_i.vs2,
            eew      : pe_req_i.eew_vs2,
            conv     : pe_req_i.conversion_vs2,
            target_fu: MFPU_ADDRGEN,
            vl       : pe_req_i.vl / NrLanes,
            scale_vl : pe_req_i.scale_vl,
            vstart   : vfu_operation_d.vstart,
            vtype    : pe_req_i.vtype,
            hazard   : pe_req_i.hazard_vs2 | pe_req_i.hazard_vd,
            default  : '0
          };
          // Since this request goes outside of the lane, we might need to request an
          // extra operand regardless of whether it is valid in this lane or not.
          if (operand_request_i[SlideAddrGenA].vl * NrLanes != pe_req_i.vl)
            operand_request_i[SlideAddrGenA].vl += 1;
          operand_request_push[SlideAddrGenA] = pe_req_i.op == VSXE;
        end

        VFU_SlideUnit: begin
          operand_request_i[SlideAddrGenA] = '{
            id       : pe_req.id,
            vs       : pe_req.vs2,
            eew      : pe_req.eew_vs2,
            conv     : pe_req.conversion_vs2,
            target_fu: ALU_SLDU,
            scale_vl : pe_req.scale_vl,
            vtype    : pe_req.vtype,
            vstart   : vfu_operation_d.vstart,
            hazard   : pe_req.hazard_vs2 | pe_req.hazard_vd,
            default  : '0
          };
          operand_request_push[SlideAddrGenA] = pe_req.use_vs2;

          unique case (pe_req.op)
            VSLIDEUP: begin
              // We need to trim full words from the end of the vector that are not used
              // as operands by the slide unit.
              // Since this request goes outside of the lane, we might need to request an
              // extra operand regardless of whether it is valid in this lane or not.
              operand_request_i[SlideAddrGenA].vl =
              (pe_req.vl - pe_req.stride + NrLanes - 1) / NrLanes;
            end
            VSLIDEDOWN: begin
              // Extra elements to ask, because of the stride
              logic [$clog2(8*NrLanes)-1:0] extra_stride;
              // Need one bit more than vl, since we will also add the stride contribution
              logic [$size(pe_req.vl):0] vl_tot;

              // We need to trim full words from the start of the vector that are not used
              // as operands by the slide unit.
              operand_request_i[SlideAddrGenA].vstart = pe_req.stride / NrLanes;

              // The stride move the initial address in boundaries of 8*NrLanes Byte.
              // If the stride is not multiple of a full VRF word (8*NrLanes Byte),
              // we must request it as well from the VRF

              // Find the number of extra elements to ask, related to the stride
              unique case (pe_req.eew_vs2)
                EW8 : extra_stride = pe_req.stride[$clog2(8*NrLanes)-1:0];
                EW16: extra_stride = {1'b0, pe_req.stride[$clog2(4*NrLanes)-1:0]};
                EW32: extra_stride = {2'b0, pe_req.stride[$clog2(2*NrLanes)-1:0]};
                EW64: extra_stride = {3'b0, pe_req.stride[$clog2(1*NrLanes)-1:0]};
                default:
                  extra_stride = {3'b0, pe_req.stride[$clog2(1*NrLanes)-1:0]};
              endcase

              // Find the total number of elements to be asked
              vl_tot = pe_req.vl;
              if (!pe_req.use_scalar_op)
                vl_tot += extra_stride;

              // Ask the elements, and ask one more if we do not perfectly divide NrLanes
              operand_request_i[SlideAddrGenA].vl = vl_tot / NrLanes;
              if (operand_request_i[SlideAddrGenA].vl * NrLanes != vl_tot)
                operand_request_i[SlideAddrGenA].vl += 1;
            end
            default:;
          endcase

          // This vector instruction uses masks
          operand_request_i[MaskM] = '{
            id     : pe_req.id,
            vs     : VMASK,
            eew    : pe_req.vtype.vsew,
            vtype  : pe_req.vtype,
            vstart : vfu_operation_d.vstart,
            hazard : pe_req.hazard_vm | pe_req.hazard_vd,
            default: '0
          };
          operand_request_push[MaskM] = !pe_req.vm;

          case (pe_req.op)
            VSLIDEUP: begin
              // We need to trim full words from the end of the vector that are not used
              // as operands by the slide unit.
              // Since this request goes outside of the lane, we might need to request an
              // extra operand regardless of whether it is valid in this lane or not.
              operand_request_i[MaskM].vl =
              ((pe_req.vl - pe_req.stride + NrLanes - 1) / 8 / NrLanes)
              >> int'(pe_req.vtype.vsew);

              if (((operand_request_i[MaskM].vl + pe_req.stride) <<
                    int'(pe_req.vtype.vsew) * NrLanes * 8 != pe_req.vl))
                operand_request_i[MaskM].vl += 1;

              // SLIDEUP only uses mask bits whose indices are > stride
              // Don't send the previous (unused) ones to the MASKU
              if (pe_req.stride >= NrLanes * 64)
                operand_request_i[MaskM].vstart += ((pe_req.stride >> NrLanes * 64) << NrLanes * 64) / 8;
            end
            VSLIDEDOWN: begin
              // Since this request goes outside of the lane, we might need to request an
              // extra operand regardless of whether it is valid in this lane or not.
              operand_request_i[MaskM].vl = ((pe_req.vl / NrLanes / 8) >> int'(
                    pe_req.vtype.vsew));
              if ((operand_request_i[MaskM].vl << int'(pe_req.vtype.vsew)) *
                  NrLanes * 8 != pe_req.vl)
                operand_request_i[MaskM].vl += 1;
            end
          endcase
        end
        VFU_MaskUnit: begin
          operand_request_i[AluA] = '{
            id      : pe_req.id,
            vs      : pe_req.vs1,
            eew     : pe_req.eew_vs1,
            scale_vl: pe_req.scale_vl,
            vtype   : pe_req.vtype,
            vstart  : vfu_operation_d.vstart,
            hazard  : pe_req.hazard_vs1 | pe_req.hazard_vd,
            default : '0
          };

          // This is an operation that runs normally on the ALU, and then gets *condensed* and
          // reshuffled at the Mask Unit.
          if (pe_req.op inside {[VMSEQ:VMSBC]}) begin
            operand_request_i[AluA].vl = vfu_operation_d.vl;
          end
          // This is an operation that runs normally on the ALU, and then gets reshuffled at the
          // Mask Unit.
          else begin
            // Since this request goes outside of the lane, we might need to request an
            // extra operand regardless of whether it is valid in this lane or not.
            operand_request_i[AluA].vl = (pe_req.vl / NrLanes) >>
            (int'(EW64) - int'(pe_req.eew_vs1));
            if ((operand_request_i[AluA].vl << (int'(EW64) - int'(pe_req.eew_vs1))) * NrLanes !=
                pe_req.vl) operand_request_i[AluA].vl += 1;
          end
          operand_request_push[AluA] = pe_req.use_vs1 && !(pe_req.op inside {[VMFEQ:VMFGE]});

          operand_request_i[AluB] = '{
            id      : pe_req.id,
            vs      : pe_req.vs2,
            eew     : pe_req.eew_vs2,
            scale_vl: pe_req.scale_vl,
            vtype   : pe_req.vtype,
            vstart  : vfu_operation_d.vstart,
            hazard  : pe_req.hazard_vs2 | pe_req.hazard_vd,
            default : '0
          };
          // This is an operation that runs normally on the ALU, and then gets *condensed* and
          // reshuffled at the Mask Unit.
          if (pe_req.op inside {[VMSEQ:VMSBC]}) begin
            operand_request_i[AluB].vl = vfu_operation_d.vl;
          end
          // This is an operation that runs normally on the ALU, and then gets reshuffled at the
          // Mask Unit.
          else begin
            // Since this request goes outside of the lane, we might need to request an
            // extra operand regardless of whether it is valid in this lane or not.
            operand_request_i[AluB].vl = (pe_req.vl / NrLanes) >>
            (int'(EW64) - int'(pe_req.eew_vs2));
            if ((operand_request_i[AluB].vl << (int'(EW64) - int'(pe_req.eew_vs2))) * NrLanes !=
                pe_req.vl) operand_request_i[AluB].vl += 1;
          end
          operand_request_push[AluB] = pe_req.use_vs2 && !(pe_req.op inside {[VMFEQ:VMFGE]});

          operand_request_i[MulFPUA] = '{
            id      : pe_req.id,
            vs      : pe_req.vs1,
            eew     : pe_req.eew_vs1,
            scale_vl: pe_req.scale_vl,
            vtype   : pe_req.vtype,
            vstart  : vfu_operation_d.vstart,
            hazard  : pe_req.hazard_vs1 | pe_req.hazard_vd,
            default : '0
          };

          // This is an operation that runs normally on the ALU, and then gets *condensed* and
          // reshuffled at the Mask Unit.
          operand_request_i[MulFPUA].vl = vfu_operation_d.vl;
          operand_request_push[MulFPUA] = pe_req.use_vs1 && pe_req.op inside {[VMFEQ:VMFGE]};

          operand_request_i[MulFPUB] = '{
            id      : pe_req.id,
            vs      : pe_req.vs2,
            eew     : pe_req.eew_vs2,
            scale_vl: pe_req.scale_vl,
            vtype   : pe_req.vtype,
            vstart  : vfu_operation_d.vstart,
            hazard  : pe_req.hazard_vs2 | pe_req.hazard_vd,
            default : '0
          };
          // This is an operation that runs normally on the ALU, and then gets *condensed* and
          // reshuffled at the Mask Unit.
          operand_request_i[MulFPUB].vl = vfu_operation_d.vl;
          operand_request_push[MulFPUB] = pe_req.use_vs2 && pe_req.op inside {[VMFEQ:VMFGE]};

          operand_request_i[MaskB] = '{
            id      : pe_req.id,
            vs      : pe_req.vd,
            eew     : pe_req.eew_vd_op,
            scale_vl: pe_req.scale_vl,
            vtype   : pe_req.vtype,
            // Since this request goes outside of the lane, we might need to request an
            // extra operand regardless of whether it is valid in this lane or not.
            vl      : (pe_req.vl / NrLanes / ELEN) << (int'(EW64) - int'(pe_req.vtype.vsew)),
            vstart  : vfu_operation_d.vstart,
            hazard  : pe_req.hazard_vd,
            default : '0
          };
          if (((pe_req.vl / NrLanes / ELEN) * NrLanes * ELEN) !=
            pe_req.vl) operand_request_i[MaskB].vl += 1;
          operand_request_push[MaskB] = pe_req.use_vd_op;

          operand_request_i[MaskM] = '{
            id     : pe_req.id,
            vs     : VMASK,
            eew    : pe_req.vtype.vsew,
            vtype  : pe_req.vtype,
            // Since this request goes outside of the lane, we might need to request an
            // extra operand regardless of whether it is valid in this lane or not.
            vl     : (pe_req.vl / NrLanes / ELEN),
            vstart : vfu_operation_d.vstart,
            hazard : pe_req.hazard_vm,
            default: '0
          };
          if ((operand_request_i[MaskM].vl * NrLanes * ELEN) != pe_req.vl) begin
            operand_request_i[MaskM].vl += 1;
          end
          operand_request_push[MaskM] = !pe_req.vm;
        end
        VFU_None: begin
          operand_request_i[MaskB] = '{
            id         : pe_req.id,
            vs         : pe_req.vs2,
            eew        : pe_req.eew_vs2,
            conv       : pe_req.conversion_vs2,
            scale_vl   : pe_req.scale_vl,
            cvt_resize : pe_req.cvt_resize,
            vtype      : pe_req.vtype,
            vl         : vfu_operation_d.vl,
            vstart     : vfu_operation_d.vstart,
            hazard     : pe_req.hazard_vs2,
            default    : '0
          };
          operand_request_push[MaskB] = 1'b1;
        end
        default:;
      endcase
    end
  end: sequencer

  always_ff @(posedge clk_i or negedge rst_ni) begin: p_sequencer_ff
    if (!rst_ni) begin
      vinsn_done_q    <= '0;
      vinsn_running_q <= '0;

      vfu_operation_o       <= '0;
      vfu_operation_valid_o <= 1'b0;

      alu_vinsn_done_o  <= 1'b0;
      mfpu_vinsn_done_o <= 1'b0;
    end else begin
      vinsn_done_q    <= vinsn_done_d;
      vinsn_running_q <= vinsn_running_d;

      vfu_operation_o       <= vfu_operation_d;
      vfu_operation_valid_o <= vfu_operation_valid_d;

      alu_vinsn_done_o  <= alu_vinsn_done_d;
      mfpu_vinsn_done_o <= mfpu_vinsn_done_d;
    end
  end

endmodule : lane_sequencer
