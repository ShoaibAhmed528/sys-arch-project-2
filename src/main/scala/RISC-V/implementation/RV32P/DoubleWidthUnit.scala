package RISCV.implementation.RV32P

import chisel3._
import chisel3.util._

import RISCV.interfaces.generic.AbstractExecutionUnit
import RISCV.model.InstructionSets
import RISCV.model.STALL_REASON
import RISCV.model.TRAP_REASON
import RISCV.model.RISCV_TYPE
import RISCV.model.RISCV_FORMAT

class DoubleWidthUnit extends AbstractExecutionUnit {

  io.misa := "b01__0000__0_00000_00000_00000_10000_00000".U

  // default safe states for all outputs.
  io.valid := false.B
  io.stall := STALL_REASON.NO_STALL

  io_pc.pc_we := false.B
  io_pc.pc_wdata := 0.U

  io_data.data_req := false.B
  io_data.data_addr := 0.U
  io_data.data_be := 0.U
  io_data.data_we := false.B
  io_data.data_wdata := 0.U

  io_trap.trap_valid := false.B
  io_trap.trap_reason := TRAP_REASON.NONE

  io_reg.reg_write_en := false.B
  io_reg.reg_write_data := 0.U

  // shorthands for wires.
  val instr = io.instr
  val instr_type = io.instr_type
  val rs1 = io_reg.reg_read_data1
  val rs2 = io_reg.reg_read_data2

  // check if instruc belongs to double width set.
  val isDoubleWidth = InstructionSets.DoubleWidthPacked
    .map(_ === instr_type)
    .foldLeft(false.B)(_ || _)

  io.valid := isDoubleWidth

  //  setting up FSM states.
  val s_LOW :: s_HIGH :: s_ACCUM_LOW :: s_ACCUM_HIGH :: Nil = Enum(4)
  val state = RegInit(s_LOW)

  // registers for holding operands across cycles.
  val saved_rs1 = RegInit(0.U(32.W))
  val saved_rs2 = RegInit(0.U(32.W))
  val saved_rd_high = RegInit(0.U(32.W))

  // raw register fields from instruc encoding.
  val raw_rs1 = instr(19, 15)
  val raw_rs2 = instr(24, 20)
  val base_rd = instr(11, 7) & "b11110".U // force even alignment.

  // instruc type flags.
  val isImmediateInstr = (instr_type === RISCV_TYPE.pli_db ||
    instr_type === RISCV_TYPE.pli_dh ||
    instr_type === RISCV_TYPE.plui_dh)

  val isWordToDouble =
    (instr_type === RISCV_TYPE.pwadd_b || instr_type === RISCV_TYPE.pwadd_h ||
      instr_type === RISCV_TYPE.pwadda_b || instr_type === RISCV_TYPE.pwadda_h)

  val isAccumulate =
    (instr_type === RISCV_TYPE.pwadda_b || instr_type === RISCV_TYPE.pwadda_h)

  val isScalarRs2 =
    (instr_type === RISCV_TYPE.padd_dbs || instr_type === RISCV_TYPE.padd_dhs || instr_type === RISCV_TYPE.padd_dws)

  // alignment for rs1/rs2 base addresses.
  val base_rs1 = Mux(isWordToDouble, raw_rs1, raw_rs1 & "b11110".U)
  val base_rs2 =
    Mux(isWordToDouble || isScalarRs2, raw_rs2, raw_rs2 & "b11110".U)

  val rs1_addr = WireDefault(0.U(5.W))
  val rs2_addr = WireDefault(0.U(5.W))
  val rd_addr = WireDefault(0.U(5.W))

  // we route the register addresses depending on FSM state.
  when(isAccumulate) {
    switch(state) {
      is(s_LOW) {
        rs1_addr := raw_rs1
        rs2_addr := raw_rs2
        rd_addr := base_rd
      }
      is(s_ACCUM_LOW) {
        rs1_addr := base_rd       // lower half of dest. (even reg)
        rs2_addr := base_rd + 1.U // upper half of dest. (odd reg)
        rd_addr := base_rd
      }
      is(s_ACCUM_HIGH) {
        rs1_addr := 0.U
        rs2_addr := 0.U
        rd_addr := base_rd + 1.U
      }
    }
  }.otherwise {
    rs1_addr := Mux(
      isImmediateInstr,
      0.U,
      Mux(state === s_HIGH && !isWordToDouble, base_rs1 + 1.U, base_rs1)
    )
    rs2_addr := Mux(
      isImmediateInstr,
      0.U,
      Mux(
        state === s_HIGH && !isWordToDouble && !isScalarRs2,
        base_rs2 + 1.U,
        base_rs2
      )
    )
    rd_addr := Mux(state === s_HIGH, base_rd + 1.U, base_rd)
  }

  io_reg.reg_rs1 := rs1_addr
  io_reg.reg_rs2 := rs2_addr
  io_reg.reg_rd := rd_addr

  // pick correct operands depending on whether we're in accum cycle or not.
  val op1_base = Mux(
    isAccumulate && (state === s_ACCUM_LOW || state === s_ACCUM_HIGH),
    saved_rs1,
    rs1
  )
  val op2_base = Mux(
    isAccumulate && (state === s_ACCUM_LOW || state === s_ACCUM_HIGH),
    saved_rs2,
    rs2
  )

  // shift by 16 on high cycle for widening ops.
  val shift_amount = Mux(
    (state === s_HIGH || state === s_ACCUM_HIGH) && isWordToDouble,
    16.U,
    0.U
  )
  val op1_active = op1_base >> shift_amount
  val op2_active = op2_base >> shift_amount

  // accumulator value depending on cycle.
  val current_rd_val = Mux(state === s_ACCUM_HIGH, saved_rd_high, rs1)
  val rd_h = VecInit.tabulate(2)(i => current_rd_val(i * 16 + 15, i * 16))

  // immediates for load instruc.
  val pli_db_imm = instr(23, 16)
  val pli_dh_imm = Cat(instr(24), instr(23, 15)).asSInt
  val plui_dh_imm = instr(24, 15)

  // slice operands into bytes and halfwords.
  val rs1_b = VecInit.tabulate(4)(i => op1_active(i * 8 + 7, i * 8))
  val rs2_b = VecInit.tabulate(4)(i => op2_active(i * 8 + 7, i * 8))
  val rs1_h = VecInit.tabulate(2)(i => op1_active(i * 16 + 15, i * 16))
  val rs2_h = VecInit.tabulate(2)(i => op2_active(i * 16 + 15, i * 16))

  // here we do the actual arithmetic.
  val padd_b_res = VecInit.tabulate(4)(i => (rs1_b(i) +& rs2_b(i))(7, 0))
  val padd_h_res = VecInit.tabulate(2)(i => (rs1_h(i) +& rs2_h(i))(15, 0))
  val padd_w_res = (op1_active +& op2_active)(31, 0)

  val padd_bs_scalar = op2_active(7, 0)
  val padd_bs_res = VecInit.tabulate(4)(i => (rs1_b(i) +& padd_bs_scalar)(7, 0))

  val padd_hs_scalar = op2_active(15, 0)
  val padd_hs_res =
    VecInit.tabulate(2)(i => (rs1_h(i) +& padd_hs_scalar)(15, 0))

  val padd_ws_scalar = op2_active
  val padd_ws_res = (op1_active +& padd_ws_scalar)(31, 0)

  val psub_b_res = VecInit.tabulate(4)(i => (rs1_b(i) -& rs2_b(i))(7, 0))
  val psub_h_res = VecInit.tabulate(2)(i => (rs1_h(i) -& rs2_h(i))(15, 0))
  val psub_w_res = (op1_active -& op2_active)(31, 0)

  // saturating signed add.
  val psadd_b_res = VecInit.tabulate(4) { i =>
    val sum = rs1_b(i).asSInt +& rs2_b(i).asSInt
    val sat = Mux(
      sum > 127.S(9.W),
      127.S(9.W),
      Mux(sum < -128.S(9.W), -128.S(9.W), sum)
    )
    sat(7, 0).asUInt
  }

  val psadd_h_res = VecInit.tabulate(2) { i =>
    val sum = rs1_h(i).asSInt +& rs2_h(i).asSInt
    val sat = Mux(
      sum > 32767.S(17.W),
      32767.S(17.W),
      Mux(sum < -32768.S(17.W), -32768.S(17.W), sum)
    )
    sat(15, 0).asUInt
  }

  val psadd_w_res = {
    val sum = op1_active.asSInt +& op2_active.asSInt
    val sat = Mux(
      sum > 2147483647.S(33.W),
      2147483647.S(33.W),
      Mux(sum < -2147483648.S(33.W), -2147483648.S(33.W), sum)
    )
    sat(31, 0).asUInt
  }

  // averaging signed add.
  val paadd_b_res =
    VecInit.tabulate(4)(i => (rs1_b(i).asSInt +& rs2_b(i).asSInt)(8, 1).asUInt)
  val paadd_h_res =
    VecInit.tabulate(2)(i => (rs1_h(i).asSInt +& rs2_h(i).asSInt)(16, 1).asUInt)
  val paadd_w_res = (op1_active.asSInt +& op2_active.asSInt)(32, 1).asUInt

  // widening add.
  val pwadd_b_res = VecInit.tabulate(2) { i =>
    val a = rs1_b(i).asSInt.pad(16)
    val b = rs2_b(i).asSInt.pad(16)
    (a + b).asUInt(15, 0)
  }

  val pwadd_h_res = {
    val a = rs1_h(0).asSInt.pad(32)
    val b = rs2_h(0).asSInt.pad(32)
    (a + b).asUInt(31, 0)
  }

  // widening accumulate.
  val pwadda_b_res = VecInit.tabulate(2) { i =>
    val wide = rs1_b(i).asSInt.pad(16) + rs2_b(i).asSInt.pad(16)
    val acc = wide + rd_h(i).asSInt
    acc.asUInt(15, 0)
  }

  val pwadda_h_res = {
    val sum =
      rs1_h(0).asSInt.pad(32) + rs2_h(0).asSInt.pad(32) + current_rd_val.asSInt
    sum.asUInt(31, 0)
  }

  // result mux.
  val out_wire = WireDefault(0.U(32.W))
  switch(instr_type) {
    is(RISCV_TYPE.pli_db) {
      out_wire := Cat(pli_db_imm, pli_db_imm, pli_db_imm, pli_db_imm)
    }
    is(RISCV_TYPE.pli_dh) {
      val imm_h = Cat(Fill(6, pli_dh_imm(9)), pli_dh_imm.asUInt)
      out_wire := Cat(imm_h, imm_h)
    }
    is(RISCV_TYPE.plui_dh) {
      val shift_imm = pli_dh_imm.asSInt << 6
      out_wire := Cat(Fill(6, pli_dh_imm(9)), shift_imm(15, 0))
    }

    is(RISCV_TYPE.padd_db) {
      out_wire := Cat(padd_b_res(3), padd_b_res(2), padd_b_res(1), padd_b_res(0))
    }
    is(RISCV_TYPE.padd_dh) { out_wire := Cat(padd_h_res(1), padd_h_res(0)) }
    is(RISCV_TYPE.padd_dw) { out_wire := padd_w_res }

    is(RISCV_TYPE.padd_dbs) {
      out_wire := Cat(padd_bs_res(3), padd_bs_res(2), padd_bs_res(1), padd_bs_res(0))
    }
    is(RISCV_TYPE.padd_dhs) { out_wire := Cat(padd_hs_res(1), padd_hs_res(0)) }
    is(RISCV_TYPE.padd_dws) { out_wire := padd_ws_res }

    is(RISCV_TYPE.psub_db) {
      out_wire := Cat(psub_b_res(3), psub_b_res(2), psub_b_res(1), psub_b_res(0))
    }
    is(RISCV_TYPE.psub_dh) { out_wire := Cat(psub_h_res(1), psub_h_res(0)) }
    is(RISCV_TYPE.psub_dw) { out_wire := psub_w_res }

    is(RISCV_TYPE.psadd_db) {
      out_wire := Cat(psadd_b_res(3), psadd_b_res(2), psadd_b_res(1), psadd_b_res(0))
    }
    is(RISCV_TYPE.psadd_dh) { out_wire := Cat(psadd_h_res(1), psadd_h_res(0)) }
    is(RISCV_TYPE.psadd_dw) { out_wire := psadd_w_res }

    is(RISCV_TYPE.paadd_db) {
      out_wire := Cat(paadd_b_res(3), paadd_b_res(2), paadd_b_res(1), paadd_b_res(0))
    }
    is(RISCV_TYPE.paadd_dh) { out_wire := Cat(paadd_h_res(1), paadd_h_res(0)) }
    is(RISCV_TYPE.paadd_dw) { out_wire := paadd_w_res }

    is(RISCV_TYPE.pwadd_b) { out_wire := Cat(pwadd_b_res(1), pwadd_b_res(0)) }
    is(RISCV_TYPE.pwadd_h) { out_wire := pwadd_h_res }

    is(RISCV_TYPE.pwadda_b) { out_wire := Cat(pwadda_b_res(1), pwadda_b_res(0)) }
    is(RISCV_TYPE.pwadda_h) { out_wire := pwadda_h_res }
  }

  // FSM control path.
  when(isDoubleWidth) {
    switch(state) {
      is(s_LOW) {
        when(isAccumulate) {
          saved_rs1 := rs1
          saved_rs2 := rs2
          io_reg.reg_write_en := false.B
          io.stall := STALL_REASON.EXECUTION_UNIT
          state := s_ACCUM_LOW
        }.otherwise {
          io.stall := STALL_REASON.EXECUTION_UNIT
          io_reg.reg_write_en := true.B
          io_reg.reg_write_data := out_wire
          state := s_HIGH
        }
      }

      is(s_HIGH) {
        io.stall := STALL_REASON.NO_STALL
        io_reg.reg_write_en := true.B
        io_reg.reg_write_data := out_wire
        io_pc.pc_we := true.B
        io_pc.pc_wdata := io_pc.pc + 4.U
        state := s_LOW
      }

      is(s_ACCUM_LOW) {
        io.stall := STALL_REASON.EXECUTION_UNIT
        io_reg.reg_write_en := true.B
        io_reg.reg_write_data := out_wire
        saved_rd_high := rs2 // grab upper reg value via second read port.
        state := s_ACCUM_HIGH
      }

      is(s_ACCUM_HIGH) {
        io.stall := STALL_REASON.NO_STALL
        io_reg.reg_write_en := true.B
        io_reg.reg_write_data := out_wire
        io_pc.pc_we := true.B
        io_pc.pc_wdata := io_pc.pc + 4.U
        state := s_LOW
      }
    }
  }.otherwise {
    io_reg.reg_write_en := false.B
    state := s_LOW
  }
}