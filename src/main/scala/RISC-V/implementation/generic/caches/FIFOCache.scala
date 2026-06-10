package RISCV.implementation.generic.caches

import chisel3._
import chisel3.util._

import RISCV.interfaces.generic.AbstractCache
import RISCV.model.CACHE_STATE

class FIFOCache(capacity: Int) extends AbstractCache {

  // Cache storage: parallel arrays for tags (addresses) and data
  val cache_tags = RegInit(VecInit(Seq.fill(capacity)(0.U(32.W))))
  val cache_data = RegInit(VecInit(Seq.fill(capacity)(0.U(32.W))))
  val cache_valid = RegInit(VecInit(Seq.fill(capacity)(false.B)))

  // FIFO pointer: index of the oldest entry (next to evict)
  val fifo_ptr = RegInit(0.U(log2Ceil(capacity).W))

  // State register
  val state = RegInit(CACHE_STATE.IDLE)

  // Hit detection: find a valid entry whose tag matches the requested address
  val hit_vec = VecInit(
    (0 until capacity).map(i =>
      cache_valid(i) && cache_tags(i) === core_io.data_addr
    )
  )
  val hit = hit_vec.asUInt.orR
  val hit_idx = PriorityEncoder(hit_vec)

  // Default outputs
  core_io.data_gnt := false.B
  core_io.data_rdata := 0.U
  mem_io.data_req := false.B
  mem_io.data_addr := core_io.data_addr
  mem_io.data_be := core_io.data_be
  mem_io.data_we := false.B
  mem_io.data_wdata := core_io.data_wdata

  when(~io_reset.rst_n) {
    state := CACHE_STATE.IDLE
    fifo_ptr := 0.U
    for (i <- 0 until capacity) {
      cache_valid(i) := false.B
    }
  }.otherwise {
    switch(state) {

      is(CACHE_STATE.IDLE) {
        when(core_io.data_req) {
          when(!core_io.data_we) {
            // READ: check cache first
            when(hit) {
              // Hit: return masked data immediately, no memory request
              core_io.data_gnt := true.B
              core_io.data_rdata := cache_data(hit_idx) & FillInterleaved(
                8,
                core_io.data_be
              )
            }.otherwise {
              // Miss: fetch from memory
              state := CACHE_STATE.READ
            }
          }.otherwise {
            // WRITE: always forward to memory
            state := CACHE_STATE.WRITE
          }
        }
      }

      is(CACHE_STATE.READ) {
        // Issue read to memory; always fetch full word so we can cache it
        mem_io.data_req := true.B
        mem_io.data_addr := core_io.data_addr
        mem_io.data_be := "hf".U
        mem_io.data_we := false.B

        when(mem_io.data_gnt) {
          // Store in cache, evicting the oldest entry if full
          cache_tags(fifo_ptr) := core_io.data_addr
          cache_data(fifo_ptr) := mem_io.data_rdata
          cache_valid(fifo_ptr) := true.B
          fifo_ptr := Mux(fifo_ptr === (capacity - 1).U, 0.U, fifo_ptr + 1.U)

          // Return masked data to core
          core_io.data_gnt := true.B
          core_io.data_rdata := mem_io.data_rdata & FillInterleaved(
            8,
            core_io.data_be
          )
          state := CACHE_STATE.IDLE
        }
      }

      is(CACHE_STATE.WRITE) {
        // Forward write to memory
        mem_io.data_req := true.B
        mem_io.data_addr := core_io.data_addr
        mem_io.data_be := core_io.data_be
        mem_io.data_we := true.B
        mem_io.data_wdata := core_io.data_wdata

        when(mem_io.data_gnt) {
          // If address is already cached, update the cached bytes (keep FIFO position)
          for (i <- 0 until capacity) {
            when(cache_valid(i) && cache_tags(i) === core_io.data_addr) {
              val mask = FillInterleaved(8, core_io.data_be)
              cache_data(i) := (cache_data(
                i
              ) & ~mask) | (core_io.data_wdata & mask)
            }
          }

          // If address was NOT cached, add it (possibly evicting oldest)
          when(!hit) {
            cache_tags(fifo_ptr) := core_io.data_addr
            cache_data(fifo_ptr) := core_io.data_wdata
            cache_valid(fifo_ptr) := true.B
            fifo_ptr := Mux(fifo_ptr === (capacity - 1).U, 0.U, fifo_ptr + 1.U)
          }

          core_io.data_gnt := true.B
          state := CACHE_STATE.IDLE
        }
      }
    }
  }
}
