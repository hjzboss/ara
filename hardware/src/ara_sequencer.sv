// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
// Description:
// Ara's sequencer controls the ordering and the dependencies between the
// parallel vector instructions in execution.

// dispatch -> main_seq -> lane_seq -> vrf_req可以流水，一周期流出一条指令，背靠背执行指令 
// 顺序发射，将所有通道看作一个整体功能单元，按照功能单元分类来进行发射，发射出的请求连接到每个pe，由信号中的vfu字段来决定pe是否响应该请求
// 能发射的指令数取决于每个功能单元的operand queues长度，由main sequencer来判断是否能发射
// 如果本周期dipatcher发来新指令请求，会在本周起生成新的访问功能单元的请求，记录下此指令的token，会在下一周期将请求发射到对应功能单元，更新各种记分牌和计数器
// 当一条指令在功能单元中完成时也会在下一周期更新记分牌和功能单元计数器
// 记分牌包括：指令运行位置的记分牌，指令运行状态的记分牌，指令的冒险记分牌
module ara_sequencer import ara_pkg::*; import rvv_pkg::*; import cf_math_pkg::idx_width; #(
    // RVV Parameters
    parameter  int unsigned NrLanes = 1,          // Number of parallel vector lanes
    // Dependant parameters. DO NOT CHANGE!
    // Ara has NrLanes + 3 processing elements: each one of the lanes, the vector load unit, the
    // vector store unit, the slide unit, and the mask unit.
    localparam int unsigned NrPEs   = NrLanes + 4
  ) (
    input  logic                            clk_i,
    input  logic                            rst_ni,
    // Interface with Ara's dispatcher
    input  ara_req_t                        ara_req_i,
    input  logic                            ara_req_valid_i,
    output logic                            ara_req_ready_o,
    output ara_resp_t                       ara_resp_o,
    output logic                            ara_resp_valid_o,
    output logic                            ara_idle_o,
    // Interface with the processing elements
    output pe_req_t                         pe_req_o,
    output logic                            pe_req_valid_o,
    output logic              [NrVInsn-1:0] pe_vinsn_running_o,
    input  logic                [NrPEs-1:0] pe_req_ready_i,
    input  pe_resp_t            [NrPEs-1:0] pe_resp_i,
    //input  logic                            alu_vinsn_done_i,
    input  logic [NrLanes-1:0]              alu_vinsn_done_i, // update: 此处更改为了接收四条lane的alu信号
    input  logic                            mfpu_vinsn_done_i,
    // Interface with the operand requesters
    // 发送给operand requester，由其进行指令调度与冒险控制
    output logic [NrVInsn-1:0][NrVInsn-1:0] global_hazard_table_o,
    // Only the slide unit can answer with a scalar response
    input  elen_t                           pe_scalar_resp_i,
    input  logic                            pe_scalar_resp_valid_i,
    output logic                            pe_scalar_resp_ready_o,
    // Interface with the Address Generation
    input  logic                            addrgen_ack_i,
    input  logic                            addrgen_error_i,
    input  vlen_t                           addrgen_error_vl_i
  );

  ///////////////////////////////////
  //  Running vector instructions  //
  ///////////////////////////////////

  // A set bit indicates that the corresponding vector instruction is running at that PE.
  // 用一个二维表记录并行的指令在哪个pe执行，如下图所示
  /*
                                     NrVInsn                                             
            ┌──────┬──────┬─────┬─────┬────────────────────────────┬─────┐               
            │      │ insn0│insn1│insn2│      ...                   │insn7│               
            ├──────┼──────┼─────┼─────┼────────────────────────────┼─────┤               
            │Lane0 │      │     │     │                            │     │               
            ├──────┼──────┼─────┼─────┼────────────────────────────┼─────┤               
            │Lane1 │      │     │     │                            │     │               
            ├──────┼──────┼─────┼─────┼────────────────────────────┼─────┤               
            │Lane2 │      │     │     │                            │     │               
            ├──────┼──────┼─────┼─────┼────────────────────────────┼─────┤               
            │      │      │     │     │                            │     │               
            │  .   │      │     │     │                            │     │               
            │  .   │      │     │     │                            │     │               
      NrPEs │  .   │      │     │     │                            │     │               
            │  .   │      │     │     │                            │     │               
  NrLanes+4 │      │      │     │     │                            │     │               
            │      │      │     │     │                            │     │               
            ├──────┼──────┼─────┼─────┼────────────────────────────┼─────┤               
            │LaneN │      │     │     │                            │     │               
            ├──────┼──────┼─────┼─────┼────────────────────────────┼─────┤               
            │ LDU  │      │     │     │                            │     │               
            ├──────┼──────┼─────┼─────┼────────────────────────────┼─────┤               
            │STDU  │      │     │     │                            │     │               
            ├──────┼──────┼─────┼─────┼────────────────────────────┼─────┤               
            │MASKU │      │     │     │                            │     │               
            ├──────┼──────┼─────┼─────┼────────────────────────────┼─────┤               
            │SLDU  │      │     │     │                            │     │               
            └──────┴──────┴─────┴─────┴────────────────────────────┴─────┘               
  */
  logic [NrPEs-1:0][NrVInsn-1:0] pe_vinsn_running_d, pe_vinsn_running_q;

  // A set bit indicates that the corresponding vector instruction in running somewhere in Ara.
  logic [NrVInsn-1:0] vinsn_running_d, vinsn_running_q;
  vid_t               vinsn_id_n;
  logic               vinsn_running_full;

  // NrLanes bits that indicate if the sequencer must stall because of a lane desynchronization.
  logic [NrVInsn-1:0] stall_lanes_desynch_vec;
  logic               stall_lanes_desynch;
  // Transpose the matrix, as vertical slices are not allowed in System Verilog
  logic [NrVInsn-1:0][NrPEs-1:0] pe_vinsn_running_q_trns;

  // Ara is idle if no instruction is currently running on it.
  assign ara_idle_o = !(|vinsn_running_q);

  // 计算出指令的id，实际上是数低位1的个数，相当于找出最低一个0的位置给新指令
  lzc #(.WIDTH(NrVInsn)) i_next_id (
    .in_i   (~vinsn_running_q  ),
    .cnt_o  (vinsn_id_n        ),
    .empty_o(vinsn_running_full)
  );

  // 记录所有正在运行的指令
  always_comb begin: p_vinsn_running
    vinsn_running_d = '0;
    for (int unsigned pe = 0; pe < NrPEs; pe++) vinsn_running_d |= pe_vinsn_running_d[pe];
  end: p_vinsn_running

  always_ff @(posedge clk_i or negedge rst_ni) begin: p_vinsn_running_ff
    if (!rst_ni) begin
      vinsn_running_q    <= '0;
      pe_vinsn_running_q <= '0;
    end else begin
      vinsn_running_q    <= vinsn_running_d;
      pe_vinsn_running_q <= pe_vinsn_running_d;
    end
  end

  assign pe_vinsn_running_o = vinsn_running_q;

  // Transpose the matrix
  for (genvar r = 0; r < NrVInsn; r++) begin : gen_trans_mtx_r
    for (genvar c = 0; c < NrPEs; c++) begin : gen_trans_mtx_c
      assign pe_vinsn_running_q_trns[r][c] = pe_vinsn_running_q[c][r];
    end
  end

  // 这样会使得通道0先执行完时，也不能发射后续指令，要等待其他通道全部执行完这条指令才可以接受新的指令
  // Stall the sequencer if the lanes get de-synchronized
  // and lane 0 is no more the last lane to finish the operation.
  // This is because the instruction counters for ALU and MFPU refers
  // to lane 0. If lane 0 finishes before the other lanes, the counter
  // is not reflecting the real lane situations anymore.
  for (genvar i = 0; i < NrVInsn; i++) begin : gen_stall_lane_desynch
    // 第i条指令没有在通道0执行，但在其他通道执行，进行停顿
    assign stall_lanes_desynch_vec[i] = ~pe_vinsn_running_q[0][i] & |pe_vinsn_running_q_trns[i][NrLanes-1:1];
  end
  assign stall_lanes_desynch = |stall_lanes_desynch_vec;

  /////////////////////////
  // Global Hazard table //
  /////////////////////////

  // Global table of the dependencies between instructions
  //
  // The row at index N is the hazard vector belonging to instruction N
  // It indicates all the instruction on which instruction N depends
  //
  // For example, with the following table, instruction 3 depends on
  // instruction 0 and instruction 2
  //
  // +--------+--------+--------+--------+--------+
  // |   -    | Insn 0 | Insn 1 | Insn 2 | Insn 3 |
  // +--------+--------+--------+--------+--------+
  // | Insn 0 |      0 |      0 |      0 |      0 |
  // | Insn 1 |      1 |      0 |      0 |      0 |
  // | Insn 2 |      0 |      0 |      0 |      0 |
  // | Insn 3 |      1 |      0 |      1 |      0 |
  // +--------+--------+--------+--------+--------+
  //
  // This information is forwarded to the operand requesters of each lane

  // 该表由每条指令中的vs1、vs2、vd、vm的依赖向量相或得出
  logic [NrVInsn-1:0][NrVInsn-1:0] global_hazard_table_d;

  /////////////////
  //  Sequencer  //
  /////////////////

  // If the instruction requires an answer to Ariane, the sequencer needs to wait.
  enum logic { IDLE, WAIT } state_d, state_q;

  // For hazard detection, we need to know which vector instruction is reading/writing to each
  // vector register
  // 跟踪32个向量寄存器的读写，只跟踪正在运行的指令，即vinsn_running_q中的指令
  typedef struct packed {
    vid_t vid;
    logic valid;
  } vreg_access_t;
  vreg_access_t [31:0] read_list_d, read_list_q;
  vreg_access_t [31:0] write_list_d, write_list_q;

  pe_req_t pe_req_d;
  logic    pe_req_valid_d;

  // This function determines the VFU responsible for handling this operation.
  function automatic vfu_e vfu(ara_op_e op);
    unique case (op) inside
      [VADD:VWREDSUM]      : vfu = VFU_Alu;
      [VMUL:VFWREDOSUM]    : vfu = VFU_MFpu;
      [VMFEQ:VMXNOR]       : vfu = VFU_MaskUnit;
      [VLE:VLXE]           : vfu = VFU_LoadUnit;
      [VSE:VSXE]           : vfu = VFU_StoreUnit;
      [VSLIDEUP:VSLIDEDOWN]: vfu = VFU_SlideUnit;
      [VMVXS:VFMVFS]       : vfu = VFU_None;
    endcase
  endfunction : vfu

  // This function determines all the targets VFUs of this operation and returns
  // a vector. Asserted bits correspond to target VFUs. Unluckily, Verilator does
  // not support assignment patterns with enum types on the indices
  function automatic logic [NrVFUs-1:0] target_vfus(ara_op_e op);
    target_vfus = '0;
    unique case (op) inside
      [VADD:VFMVSF]:
        for (int i = 0; i < NrVFUs; i++)
          if (i == VFU_Alu) target_vfus[i] = 1'b1;
      [VREDSUM:VWREDSUM]:
        for (int i = 0; i < NrVFUs; i++)
          if (i == VFU_Alu || i == VFU_SlideUnit) target_vfus[i] = 1'b1;
      [VFREDUSUM:VFWREDOSUM]:
        for (int i = 0; i < NrVFUs; i++)
          if (i == VFU_MFpu || i == VFU_SlideUnit) target_vfus[i] = 1'b1;
      [VMUL:VFCVTFF]:
        for (int i = 0; i < NrVFUs; i++)
          if (i == VFU_MFpu) target_vfus[i] = 1'b1;
      [VMSEQ:VMXNOR]:
        for (int i = 0; i < NrVFUs; i++)
          if (i == VFU_Alu || i == VFU_MaskUnit) target_vfus[i] = 1'b1;
      [VMFEQ:VMFGE]:
        for (int i = 0; i < NrVFUs; i++)
          if (i == VFU_MFpu || i == VFU_MaskUnit) target_vfus[i] = 1'b1;
      [VLE:VLXE]:
        for (int i = 0; i < NrVFUs; i++)
          if (i == VFU_LoadUnit) target_vfus[i] = 1'b1;
      [VSE:VSXE]:
        for (int i = 0; i < NrVFUs; i++)
          if (i == VFU_StoreUnit) target_vfus[i] = 1'b1;
      [VSLIDEUP:VSLIDEDOWN]:
        for (int i = 0; i < NrVFUs; i++)
          if (i == VFU_SlideUnit) target_vfus[i] = 1'b1;
      [VMVXS:VFMVFS]:
        for (int i = 0; i < NrVFUs; i++)
          if (i == VFU_None) target_vfus[i] = 1'b1;
    endcase
  endfunction : target_vfus

  localparam int unsigned InsnQueueDepth [NrVFUs] = '{
    ValuInsnQueueDepth,
    MfpuInsnQueueDepth,
    SlduInsnQueueDepth,
    MaskuInsnQueueDepth,
    VlduInsnQueueDepth,
    VstuInsnQueueDepth,
    NoneInsnQueueDepth
  };

  logic ara_req_token_d, ara_req_token_q;

  // Counters keep track of how many instructions each unit is running.
  // They have the same size only to keep the code easy.
  logic [NrVFUs-1:0] [idx_width(MaxVInsnQueueDepth + 1)-1:0] insn_queue_cnt_q;
  logic [NrVFUs-1:0] insn_queue_done;
  logic [NrVFUs-1:0] insn_queue_cnt_en, insn_queue_cnt_down, insn_queue_cnt_up;
  // Each FU has its own ready signal
  logic [NrVFUs-1:0] vinsn_queue_ready;
  // Bit [i] is 1'b1 if the respective PE is ready for the issue of this insn
  logic [NrVFUs-1:0] vinsn_queue_issue;
  logic              accepted_insn, accepted_insn_stalled;
  logic [NrVFUs-1:0] target_vfus_vec;
  // Gold tickets and passes
  // Normally, instructions can be issued to the lane sequencer only if
  // the counters have not reached their maximum capacity.
  // When an instruction enters the main sequencer, it can happen that the
  // counter is already at maximum capacity The instruction is
  // registered anyway taking the counter beyond the maximum capacity.
  // In this case, the instruction will get a gold ticket, to witness that
  // it was already registered with the counter, so that the instruction can
  // pass the checks when the counter returns within its limits, even if
  // it is at its maximum capacity (the instruction was already counted!)
  logic [NrVFUs-1:0] gold_ticket_d, gold_ticket_q;
  logic [NrVFUs-1:0] priority_pass;

  // Signal to know if there is a mask instruction being executed by the MASKU
  // that can use the MaskB operand queue. If there is a running MASKU instruction,
  // we cannot sample the scalar operand.
  // Since the scalar move uses the MaskB opqueue, we need to wait to finish
  // the MASKU insn to be sure that the forwarded value is the scalar one
  logic running_mask_insn_d, running_mask_insn_q;

  // pe_req_ready_i comes from all the lanes
  // It is deasserted if the current request is stuck
  // because the target operand requesters are not ready in that lane
  logic [NrLanes-1:0] operand_requester_ready;
  assign operand_requester_ready = pe_req_ready_i[NrLanes-1:0];

  // Update the token only upon new instructions
  assign ara_req_token_d = (ara_req_valid_i) ? ara_req_i.token : ara_req_token_q;

  // 状态机的转换
  // 主要部分：冒险检测，计分板设置，pe请求生成，发射指令，寄存器状态跟踪，判断是否能继续接受下一条指令
  // 当是加载或者存储指令时，需要等待地址转换完成才可以接受下一条派遣指令
  always_comb begin: p_sequencer
    // Default assignments
    state_d               = state_q;
    pe_vinsn_running_d    = pe_vinsn_running_q;
    read_list_d           = read_list_q;
    write_list_d          = write_list_q;
    global_hazard_table_d = global_hazard_table_o;

    // Maintain request
    pe_req_d       = '0;
    pe_req_valid_d = 1'b0;

    // No response
    ara_resp_o       = '0;
    ara_resp_valid_o = 1'b0;

    // Always ready to receive a new request
    ara_req_ready_o = 1'b1;

    // Not ready by default
    pe_scalar_resp_ready_o = 1'b0;

    // Update vector register's access list
    // 当指令执行完毕时在下一周期清空其所跟踪的寄存器的读写状态
    for (int unsigned v = 0; v < 32; v++) begin
      read_list_d[v].valid &= vinsn_running_q[read_list_q[v].vid] ;
      write_list_d[v].valid &= vinsn_running_q[write_list_q[v].vid];
    end

    // Update the running vector instructions
    // 当pe中一条指令完成时清空二维表中pe对应指令位
    // 实际上要比真正写回的时间延一拍
    for (int pe = 0; pe < NrPEs; pe++) pe_vinsn_running_d[pe] &= ~pe_resp_i[pe].vinsn_done;

    case (state_q)
      IDLE: begin
        // Sent a request, but the operand requesters are not ready
        // Do not trap here the instructions that do not need any operands at all
        // 当向pe发送了请求但是pe没有准备好，则状态不变，不再从dispathcer接受新指令
        if (pe_req_valid_o && !(&operand_requester_ready || (is_load(pe_req_o.op) && pe_req_o.vm))) begin
          // Maintain output
          pe_req_d               = pe_req_o;
          pe_req_valid_d         = pe_req_valid_o;

          // We are not ready
          ara_req_ready_o = 1'b0;
        // Received a new request
        end else if (ara_req_valid_i) begin
          // The target PE is ready, and we can handle another running vector instruction
          // Let instructions with priority pass be issued
          // 要等目标VFU ready，而且所有指令都要保证是通道0先执行完，才可以继续发射新指令
          // 此处的队列是按VFU分类的，而不是lane和vlsu这些
          if (&vinsn_queue_issue && !stall_lanes_desynch && !vinsn_running_full) begin
            ///////////////
            //  Hazards  //
            ///////////////

            // RAW
            // hazard_vs1记录vs1依赖于哪条指令，根据指令的id来索引，如hazard_vs1[2] = 1说明该指令的vs1的产生依赖于vid=2的指令
            if (ara_req_i.use_vs1) pe_req_d.hazard_vs1[write_list_d[ara_req_i.vs1].vid] |=
              write_list_d[ara_req_i.vs1].valid;
            if (ara_req_i.use_vs2) pe_req_d.hazard_vs2[write_list_d[ara_req_i.vs2].vid] |=
              write_list_d[ara_req_i.vs2].valid;
            if (!ara_req_i.vm) pe_req_d.hazard_vm[write_list_d[VMASK].vid] |=
              write_list_d[VMASK].valid;

            // WAR
            // TODO: 为什么hazard_vs1和hazard_vs2设置的位置一样
            if (ara_req_i.use_vd) begin
              pe_req_d.hazard_vs1[read_list_d[ara_req_i.vd].vid] |= read_list_d[ara_req_i.vd].valid;
              pe_req_d.hazard_vs2[read_list_d[ara_req_i.vd].vid] |= read_list_d[ara_req_i.vd].valid;
              pe_req_d.hazard_vm[read_list_d[ara_req_i.vd].vid] |= read_list_d[ara_req_i.vd].valid;
            end

            // WAW
            if (ara_req_i.use_vd) pe_req_d.hazard_vd[write_list_d[ara_req_i.vd].vid] |=
              write_list_d[ara_req_i.vd].valid;

            /////////////
            //  Issue  //
            /////////////

            // Populate the PE request
            pe_req_d = '{
              id            : vinsn_id_n,
              op            : ara_req_i.op,
              vm            : ara_req_i.vm,
              eew_vmask     : ara_req_i.eew_vmask,
              vfu           : vfu(ara_req_i.op),
              vs1           : ara_req_i.vs1,
              use_vs1       : ara_req_i.use_vs1,
              conversion_vs1: ara_req_i.conversion_vs1,
              eew_vs1       : ara_req_i.eew_vs1,
              vs2           : ara_req_i.vs2,
              use_vs2       : ara_req_i.use_vs2,
              conversion_vs2: ara_req_i.conversion_vs2,
              eew_vs2       : ara_req_i.eew_vs2,
              use_vd_op     : ara_req_i.use_vd_op,
              eew_vd_op     : ara_req_i.eew_vd_op,
              scalar_op     : ara_req_i.scalar_op,
              use_scalar_op : ara_req_i.use_scalar_op,
              swap_vs2_vd_op: ara_req_i.swap_vs2_vd_op,
              stride        : ara_req_i.stride,
              is_stride_np2 : ara_req_i.is_stride_np2,
              vd            : ara_req_i.vd,
              use_vd        : ara_req_i.use_vd,
              emul          : ara_req_i.emul,
              fp_rm         : ara_req_i.fp_rm,
              wide_fp_imm   : ara_req_i.wide_fp_imm,
              cvt_resize    : ara_req_i.cvt_resize,
              scale_vl      : ara_req_i.scale_vl,
              vl            : ara_req_i.vl,
              vstart        : ara_req_i.vstart,
              vtype         : ara_req_i.vtype,
              hazard_vd     : pe_req_d.hazard_vd,
              hazard_vm     : pe_req_d.hazard_vm,
              hazard_vs1    : pe_req_d.hazard_vs1,
              hazard_vs2    : pe_req_d.hazard_vs2,
              default       : '0
            };

            // Populate the global hazard table
            global_hazard_table_d[vinsn_id_n] = pe_req_d.hazard_vd  | pe_req_d.hazard_vm |
                                                pe_req_d.hazard_vs1 | pe_req_d.hazard_vs2;

            // We only issue instructions that take no operands if they have no hazards.
            // Moreover, SLIDE instructions cannot be always chained
            // ToDo: optimize the case for vslide1down, vslide1up (wait 2 cycles, then chain)
            if (!(|{ara_req_i.use_vs1, ara_req_i.use_vs2, ara_req_i.use_vd_op, !ara_req_i.vm}) &&
                |{pe_req_d.hazard_vs1, pe_req_d.hazard_vs2, pe_req_d.hazard_vm, pe_req_d.hazard_vd} ||
                (pe_req_d.op == VSLIDEUP && |{pe_req_d.hazard_vd, pe_req_d.hazard_vs1, pe_req_d.hazard_vs2}) ||
                (pe_req_d.op == VSLIDEDOWN && |{pe_req_d.hazard_vs1, pe_req_d.hazard_vs2}))
            begin
              ara_req_ready_o = 1'b0;
              pe_req_valid_d  = 1'b0;
            end else begin
              // Acknowledge instruction
              ara_req_ready_o = 1'b1;

              // 指令运行状态更新，将对应功能单元的该指令运行状态更新为运行
              // Remember that the vector instruction is running
              // 是以VFU的视角来进行的划分
              unique case (vfu(ara_req_i.op))
                VFU_LoadUnit : pe_vinsn_running_d[NrLanes + OffsetLoad][vinsn_id_n]  = 1'b1;
                VFU_StoreUnit: pe_vinsn_running_d[NrLanes + OffsetStore][vinsn_id_n] = 1'b1;
                VFU_SlideUnit: pe_vinsn_running_d[NrLanes + OffsetSlide][vinsn_id_n] = 1'b1;
                VFU_MaskUnit : pe_vinsn_running_d[NrLanes + OffsetMask][vinsn_id_n]  = 1'b1;
                VFU_None     : ;
                default: for (int l = 0; l < NrLanes; l++)
                    // Instruction is running on the lanes
                    // 将所有通道看作一个整体功能单元
                    pe_vinsn_running_d[l][vinsn_id_n] = 1'b1;
              endcase

              // Masked vector instructions also run on the mask unit
              // 只要使用了掩码，掩码单元就要标记为运行
              pe_vinsn_running_d[NrLanes + OffsetMask][vinsn_id_n] |= !ara_req_i.vm;

              // Some instructions need to wait for an acknowledgment
              // before being committed with Ariane
              // 当是load store指令时，需要进行地址转换，随后再发送到vldu或者vstu
              if (is_load(ara_req_i.op) || is_store(ara_req_i.op) || !ara_req_i.use_vd) begin
                ara_req_ready_o = 1'b0;
                state_d         = WAIT;
              end

              // Issue the instruction
              pe_req_valid_d = 1'b1;

              // 寄存器读写状态更新
              // Mark that this vector instruction is writing to vector vd
              if (ara_req_i.use_vd) write_list_d[ara_req_i.vd] = '{vid: vinsn_id_n, valid: 1'b1};

              // Mark that this loop is reading vs
              if (ara_req_i.use_vs1) read_list_d[ara_req_i.vs1] = '{vid: vinsn_id_n, valid: 1'b1};
              if (ara_req_i.use_vs2) read_list_d[ara_req_i.vs2] = '{vid: vinsn_id_n, valid: 1'b1};
              if (!ara_req_i.vm) read_list_d[VMASK]             = '{vid: vinsn_id_n, valid: 1'b1};
            end
          end else ara_req_ready_o = 1'b0; // Wait until the PEs are ready
        end
      end

      WAIT: begin
        // Wait until we got an answer from lane 0
        ara_req_ready_o = 1'b0;

        // Maintain output
        pe_req_d       = pe_req_o;
        pe_req_valid_d = pe_req_valid_o;

        // Consume the request if acknowledged during a scalar move
        if (pe_req_valid_o && &operand_requester_ready)
          pe_req_valid_d = 1'b0;

        // Wait for the address translation
        if ((is_load(pe_req_d.op) || is_store(pe_req_d.op)) && addrgen_ack_i) begin
          state_d             = IDLE;
          ara_req_ready_o     = 1'b1;
          ara_resp_valid_o    = 1'b1;
          ara_resp_o.error    = addrgen_error_i;
          ara_resp_o.error_vl = addrgen_error_vl_i;
        end

        // Wait for the scalar result
        if (!ara_req_i.use_vd && pe_scalar_resp_valid_i) begin
          // Acknowledge the request
          state_d                = IDLE;
          ara_req_ready_o        = 1'b1;
          ara_resp_o.resp        = pe_scalar_resp_i;
          ara_resp_valid_o       = 1'b1;
          pe_scalar_resp_ready_o = pe_scalar_resp_valid_i & ~running_mask_insn_q;
        end
      end
    endcase

    // Update the global hazard table
    // 当一条指令没有在运行时，清除其冒险记录
    for (int id = 0; id < NrVInsn; id++) global_hazard_table_d[id] &= vinsn_running_d;
  end : p_sequencer

  always_ff @(posedge clk_i or negedge rst_ni) begin: p_sequencer_ff
    if (!rst_ni) begin
      state_q <= IDLE;

      read_list_q  <= '0;
      write_list_q <= '0;

      pe_req_o       <= '0;
      pe_req_valid_o <= 1'b0;

      ara_req_token_q <= 1'b1;
      gold_ticket_q   <= 1'b0;

      global_hazard_table_o <= '0;

      running_mask_insn_q <= 1'b0;
    end else begin
      state_q <= state_d;

      read_list_q  <= read_list_d;
      write_list_q <= write_list_d;

      pe_req_o       <= pe_req_d;
      pe_req_valid_o <= pe_req_valid_d;

      ara_req_token_q <= ara_req_token_d;
      gold_ticket_q   <= gold_ticket_d;

      global_hazard_table_o <= global_hazard_table_d;

      running_mask_insn_q <= running_mask_insn_d;
    end
  end

  /////////////////
  // Scalar Move //
  /////////////////

  // This signal detects only instructions that produce
  // a mask vector, to reduce latency of scalar moves
  // if a masked vector instruction is ongoing

  // This works only if MASKU insn queue has width == 1
  always_comb begin
    running_mask_insn_d = running_mask_insn_q;

    if (|pe_resp_i[NrLanes+OffsetMask].vinsn_done)
      running_mask_insn_d = 1'b0;

    if (pe_req_valid_o && &operand_requester_ready && pe_req_o.vfu == VFU_MaskUnit)
      running_mask_insn_d = 1'b1;
  end

  //////////////
  // Counters //
  //////////////

  // Instructions are registered upon entry by the FUs insn queue counters.

  // ALU and MFPU has different signal sources
  // 当lane0的valu中在第1拍提交指令，第2拍从写回队列中清除，第2拍该信号也会被设置为高
  //assign insn_queue_done[VFU_Alu]       = 1'b1;
  assign insn_queue_done[VFU_Alu]       = alu_vinsn_done_i;
  assign insn_queue_done[VFU_MFpu]      = mfpu_vinsn_done_i;
  assign insn_queue_done[VFU_LoadUnit]  = |pe_resp_i[NrLanes+OffsetLoad].vinsn_done;
  assign insn_queue_done[VFU_StoreUnit] = |pe_resp_i[NrLanes+OffsetStore].vinsn_done;
  assign insn_queue_done[VFU_MaskUnit]  = |pe_resp_i[NrLanes+OffsetMask].vinsn_done;
  assign insn_queue_done[VFU_SlideUnit] = |pe_resp_i[NrLanes+OffsetSlide].vinsn_done;
  // Dummy counter, just for compatibility
  assign insn_queue_done[VFU_None]      = insn_queue_cnt_up[VFU_None];

  // Register the incoming instruction if it is valid
  assign accepted_insn = ara_req_valid_i & (ara_req_token_q != ara_req_i.token);

  // The new accepted instruction will not be immediately issued
  // 一般出现在状态是wait的情况，即等待地址转换的时候接收了新指令
  assign accepted_insn_stalled = accepted_insn & ~ara_req_ready_o;

  // Masked instructions do use the mask unit as well
  always_comb begin
    target_vfus_vec                = target_vfus(ara_req_i.op);
    target_vfus_vec[VFU_MaskUnit] |= ~ara_req_i.vm;
  end

  /*
  *  update: valu请求解耦合，只要没有lane满，则可以发射，不管四条lane间的执行顺序如何
  */
  // 为每个lane新增一个计数器
  logic [NrLanes-1:0] [idx_width(MaxVInsnQueueDepth + 1)-1:0] insn_queue_cnt_lane_q;
  logic [NrLanes-1:0] insn_queue_lane_cnt_en, insn_queue_lane_cnt_down, insn_queue_lane_cnt_up;
  // Each FU has its own ready signal
  logic [NrLanes-1:0] vinsn_queue_lane_ready;
  logic [NrLanes-1:0] lane_full_ready; // 每个lane是否接近满了
  logic [NrLanes-1:0] lane_full; // 是否有lane满了

  // 为每个lane生成一个计数器
  for (genvar i = 0; i < NrLanes; i++) begin: gen_lane_cnt
    // The width can be optimized for each counter
    localparam CNT_WIDTH = idx_width(MaxVInsnQueueDepth + 1);

    counter #(
        .WIDTH           (CNT_WIDTH),
        .STICKY_OVERFLOW (0)
    ) i_insn_queue_cnt (
        .clk_i           (clk_i                       ),
        .rst_ni          (rst_ni                      ),
        .clear_i         (1'b0                        ),
        .en_i            (insn_queue_lane_cnt_en[i]   ),
        .load_i          (1'b0                        ),
        .down_i          (insn_queue_lane_cnt_down[i] ),
        .d_i             ('0                          ),
        .q_o             (insn_queue_cnt_lane_q[i]    ),
        .overflow_o      (                            )
    );

    assign vinsn_queue_lane_ready[i] = insn_queue_cnt_q[i] < InsnQueueDepth[VFU_Alu];
    assign insn_queue_lane_cnt_up[i] = accepted_insn & target_vfus_vec[VFU_Alu]; // 只要alu计数器增加，每个lane的计数器就增加
    assign insn_queue_lane_cnt_down[i] = alu_vinsn_done_i[i]; // 每个lane的计数器单独减少
    assign insn_queue_lane_cnt_en[i] = insn_queue_lane_cnt_up[i] ^ insn_queue_lane_cnt_down[i];
    // 用于产生golden_tickets
    assign lane_full_ready[i] = insn_queue_cnt_lane_q[i] >= (InsnQueueDepth[VFU_Alu] - 1);
    assign lane_full[i] = insn_queue_cnt_lane_q[i] == InsnQueueDepth[VFU_Alu];
  end
  // valu的计数器
  assign vinsn_queue_ready[VFU_Alu] = &vinsn_queue_lane_ready; // 所有lane都ready才可以向valu发射一条新指令
  // 只要有一个lane快满了，就赋gold
  assign gold_ticket_d[VFU_Alu] = accepted_insn_stalled
                          ? |lane_full_ready & target_vfus_vec[VFU_Alu]
                          : gold_ticket_q[VFU_Alu];
  // 只要有一个lane满了，就是优先
  assign priority_pass[VFU_Alu] = gold_ticket_q[VFU_Alu] & |lane_full & (ara_req_token_q == ara_req_i.token);
  assign vinsn_queue_issue[VFU_Alu] = ~target_vfus_vec[VFU_Alu] | (vinsn_queue_ready[VFU_Alu] | priority_pass[VFU_Alu]);


  // One counter per VFU
  // 每个VFU发射的指令数更新，VFU发射信号的更新等等
  // TODO: alu的计数器更改为每一个lane一个，将alu一个计数器替换成了4个计数器
  for (genvar i = VFU_MFpu; i < NrVFUs; i++) begin : gen_seq_fu_cnt
    // The width can be optimized for each counter
    localparam CNT_WIDTH = idx_width(MaxVInsnQueueDepth + 1);

    counter #(
        .WIDTH           (CNT_WIDTH),
        .STICKY_OVERFLOW (0)
    ) i_insn_queue_cnt (
        .clk_i           (clk_i                 ),
        .rst_ni          (rst_ni                ),
        .clear_i         (1'b0                  ),
        .en_i            (insn_queue_cnt_en[i]  ),
        .load_i          (1'b0                  ),
        .down_i          (insn_queue_cnt_down[i]),
        .d_i             ('0                    ),
        .q_o             (insn_queue_cnt_q[i]   ),
        .overflow_o      (/* Unconnected */     )
    );

    // Each PE is ready only if it can accept a new instruction in the queue
    assign vinsn_queue_ready[i] = insn_queue_cnt_q[i] < InsnQueueDepth[i];
    // Count up on the right counter
    assign insn_queue_cnt_up[i] = accepted_insn & target_vfus_vec[i];
    // Count down if an instruction was consumed by the PE
    assign insn_queue_cnt_down[i] = insn_queue_done[i];
    // Don't count if one instruction is issued and one is consumed
    assign insn_queue_cnt_en[i] = insn_queue_cnt_up[i] ^ insn_queue_cnt_down[i];
    // 当sequencer新接受一条指令但是无法在下一周期将该指令发射（该指令被停顿），
    // 但是新指令都会让对应的队列计数器在下一周期递增，如果此时计数器离最大差1，会在下一周期将队列标记为满，导致vinsn_queue_ready为假，
    // 此时即使sequencer已经准备好发射该指令也会被停顿，导致死锁
    // 因此gold_ticket的作用就是防止被停顿的指令导致的死锁情况
    // Assign the gold ticket when:
    //   1) The new instruction finds the target cnt already full
    //   2) The new instruction is stalled and the target cnt is pre-filled
    // In both cases the instruction is stalled, and it should pass as soon as
    // insn_queue_cnt_q[i] == InsnQueueDepth[i] since it was already counted
    assign gold_ticket_d[i] = accepted_insn_stalled
                            ? (insn_queue_cnt_q[i] >= (InsnQueueDepth[i] - 1)) & target_vfus_vec[i]
                            : gold_ticket_q[i];
    // The instructions with a gold ticket can pass the checks even if the cnt is full,
    // but not when (insn_queue_cnt_q[i] == InsnQueueDepth[i] + 1)
    // Moreover, just arrived instructions cannot use the golden ticket of a previous instruction
    assign priority_pass[i] = gold_ticket_q[i] & (insn_queue_cnt_q[i] == InsnQueueDepth[i]) &
      (ara_req_token_q == ara_req_i.token);
    // The instruction queue [i] allows us to issue the instruction
    // If the insn is not targeting the PE [i], PE [i] cannot stall the instruction issue.
    // Each targeted PE must be ready (either with cnt < MAX or with a priority pass)
    // vfu的发射信号，当这条指令不使用某功能单元时，该功能单元不能阻止该指令的发射
    // 如果要使用某个功能单元，需要该功能单元ready或者pass
    assign vinsn_queue_issue[i] = ~target_vfus_vec[i] | (vinsn_queue_ready[i] | priority_pass[i]);
  end

endmodule : ara_sequencer
