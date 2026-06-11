package RISCV.implementation.RV32P

import chisel3._
import chisel3.util._

import RISCV.interfaces.generic.AbstractExecutionUnit
import RISCV.model.InstructionSets
import RISCV.model.STALL_REASON
import RISCV.model.TRAP_REASON
import RISCV.model.RISCV_TYPE

class MergeUnit extends AbstractExecutionUnit {

  io.misa := "b01__0000__0_00000_00000_00000_10000_00000".U

  // default safe states.
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
  io_reg.reg_rs1 := io.instr(19, 15)
  io_reg.reg_rs2 := io.instr(24, 20)
  io_reg.reg_rd := io.instr(11, 7)

  // shorthands for wires.
  val instr = io.instr
  val instr_type = io.instr_type
  val rs1 = io_reg.reg_read_data1
  val rs2 = io_reg.reg_read_data2

  // check if instruc is one we handle.
  val isMergeUnitInstr = (instr_type === RISCV_TYPE.mvm) ||
    (instr_type === RISCV_TYPE.mvmn) ||
    (instr_type === RISCV_TYPE.merge) ||
    (instr_type === RISCV_TYPE.pack)

  io.valid := isMergeUnitInstr

  // we set up FSM states.
  val s_LOW :: s_READ_RD :: s_WRITE_BACK :: Nil = Enum(3)
  val state = RegInit(s_LOW)

  // registers to hold operands between cycles.
  val saved_rs1 = RegInit(0.U(32.W))
  val saved_rs2 = RegInit(0.U(32.W))
  val saved_rd = RegInit(0.U(32.W))

  switch(state) {
    is(s_LOW) {
      io_reg.reg_rs1 := instr(19, 15)
      io_reg.reg_rs2 := instr(24, 20)
      io_reg.reg_rd := instr(11, 7)

      when(isMergeUnitInstr) {
        when(instr_type === RISCV_TYPE.pack) {
          // pack only needs rs1 and rs2, so we can do it in one cycle.
          io.stall := STALL_REASON.NO_STALL
          io_reg.reg_write_en := true.B
          io_reg.reg_write_data := Cat(rs2(15, 0), rs1(15, 0))
          io_pc.pc_we := true.B
          io_pc.pc_wdata := io_pc.pc + 4.U
          state := s_LOW
        }.otherwise {
          // mvm/mvmn/merge also read rd, so we need extra cycles.
          io.stall := STALL_REASON.EXECUTION_UNIT
          saved_rs1 := rs1
          saved_rs2 := rs2
          io_reg.reg_write_en := false.B
          state := s_READ_RD
        }
      }
    }

    is(s_READ_RD) {
      // we read current value of rd here.
      io.stall := STALL_REASON.EXECUTION_UNIT
      io_reg.reg_rs1 := instr(11, 7) // route rd into read port 1.
      io_reg.reg_rs2 := 0.U
      io_reg.reg_rd := instr(11, 7)
      io_reg.reg_write_en := false.B

      saved_rd := rs1
      state := s_WRITE_BACK
    }

    is(s_WRITE_BACK) {
      io.stall := STALL_REASON.NO_STALL
      io_reg.reg_rs1 := instr(11, 7)
      io_reg.reg_rs2 := 0.U
      io_reg.reg_rd := instr(11, 7)
      io_reg.reg_write_en := true.B

      val out_wire = WireDefault(0.U(32.W))
      switch(instr_type) {
        is(RISCV_TYPE.mvm) { // X[rd] = (~m & X[rd]) | (m & X[rs1])
          out_wire := (saved_rs2 & saved_rs1) | (~saved_rs2 & saved_rd)
        }
        is(RISCV_TYPE.mvmn) { // X[rd] = (~m & X[rs1]) | (m & X[rd])
          out_wire := (~saved_rs2 & saved_rs1) | (saved_rs2 & saved_rd)
        }
        is(RISCV_TYPE.merge) { // X[rd] = (~d0 & X[rs1]) | (d0 & X[rs2])
          out_wire := (saved_rd & saved_rs2) | (~saved_rd & saved_rs1)
        }
      }
      io_reg.reg_write_data := out_wire

      io_pc.pc_we := true.B
      io_pc.pc_wdata := io_pc.pc + 4.U
      state := s_LOW
    }
  }
}
