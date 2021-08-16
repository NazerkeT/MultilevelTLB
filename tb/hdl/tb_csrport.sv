// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Michael Schaffner <schaffner@iis.ee.ethz.ch>, ETH Zurich
// Date: 15.08.2018
// Description: program that emulates a cache readport. the program can generate
// randomized or linear read sequences, and it checks the returned responses against
// the expected responses coming directly from the emulated memory (tb_mem).
//

`include "tb.svh"
  
program tb_csrport  import tb_pkg::*; import ariane_pkg::*; #(
  parameter string       PortName      = "read port 0",
  parameter              FlushRate     = 1,
  parameter              TlbHitRate    = 95,
  parameter              MemWords      = 1024*1024,// in 64bit words
  parameter              RndSeed       = 1110,
  parameter              Verbose       = 0
) (
  input logic            clk_i,
  input logic            rst_ni,

  // to testbench master
  ref   string           test_name_i,
  input  logic [6:0]     req_rate_i, // a rate between 0 and 100 percent
  input  seq_t           seq_type_i,
  input  logic           tlb_rand_en_i,
  input  logic           flush_rand_en_i,
  input  logic           seq_run_i,
  input  logic [31:0]    seq_num_resp_i,
  input  logic           seq_last_i,
  output logic           seq_done_o,

  // DUT-MMU interface 
  input  logic           lsu_dtlb_hit_i,     // sent in the same cycle as the request if translation hits in the DTLB
  input  icache_areq_i_t icache_areq_i, 

  // General control signals - put default values for tb
  output riscv::priv_lvl_t               priv_lvl_o, 
  output riscv::priv_lvl_t               ld_st_priv_lvl_o,
  output logic                           sum_o,  
  output logic                           mxr_o,  
       
  // input logic flag_mprv_i,       
  output logic [riscv::PPNW-1:0]         satp_ppn_o,
  output logic [ASID_WIDTH-1:0]          asid_o,
  output logic [ASID_WIDTH-1:0]          asid_to_be_flushed_o,
  output logic [riscv::VLEN-1:0]         vaddr_to_be_flushed_o,
  output logic                           flush_tlb_o
);
  // to-do: set default values
  //        update tb_mem

  // leave this
  timeunit 1ps;
  timeprecision 1ps;

  logic [63:0] paddr, vaddr;
  logic [ASID_WIDTH-1:0] asid;
  logic seq_end_req, seq_end_ack, prog_end;
    
  // no delay between
  assign satp_ppn_o            = paddr;
  assign asid_o                = asid;  
  assign asid_to_be_flushed_o  = asid;
  assign vaddr_to_be_flushed_o = vaddr;
    
///////////////////////////////////////////////////////////////////////////////
// Helper tasks
///////////////////////////////////////////////////////////////////////////////
  
  task automatic genRandReq();
     automatic logic [63:0] val;

    void'($urandom(RndSeed));

    paddr = '0;
    asid  = '0;

    while(~seq_end_req) begin
      // randomize request
      void'(randomize(paddr) with {paddr >= 0; paddr < (MemWords<<3);});  
      void'(randomize(asid) with {asid >= 0; asid < (2**(ASID_WIDTH));});
      
      void'(randomize(val) with {val >= 0; val < 100);});
      if ((val < FlushRate) && flush_rand_en_i) begin
        void'(randomize(vaddr) with {vaddr >= 0; vaddr < (MemWords<<3);}); 
        flush_tlb_o = 1'b1;
      end

      `APPL_WAIT_COMB_SIG(clk_i, icache_areq_i.fetch_valid | lsu_dtlb_hit_i)
      
      `APPL_WAIT_CYC(clk_i,1)
    end

    paddr = '0;
    asid  = '0;

  endtask : genRandReq

  task automatic genSeqRead();
    automatic logic [63:0] paddr_val, asid_val;
    paddr = '0;
    asid  = '0;
    
    paddr_val = '0;
    asid_val  = '0;
    
    while(~seq_end_req) begin
      paddr = paddr_val;
      asid  = asid_val;
      // generate linear read
      paddr_val = (paddr_val + 8) % (MemWords<<3);
      asid_val  = (asid_val + 1) % (ASID_WIDTH);
      
      `APPL_WAIT_COMB_SIG(clk_i, icache_areq_i.fetch_valid | lsu_dtlb_hit_i)
      
      `APPL_WAIT_CYC(clk_i,1) 
    end
    icache_areq_o.fetch_req = '0;
    
  endtask : genSeqRead
    
  // ---> These are smth useful for tlbs also, test later!
  // Generate a sequence of reads to the same set (constant index)
  
//  task automatic genSetSeqRead();
//    automatic logic [63:0] val, rnd;
//    vaddr                        = CachedAddrBeg + 2 ** DCACHE_INDEX_WIDTH;
//    dut_req_port_o.data_req      = '0;
//    dut_req_port_o.data_size     = '0;
//    dut_req_port_o.kill_req      = '0;
//    val                          = CachedAddrBeg + 2 ** DCACHE_INDEX_WIDTH;
//    while(~seq_end_req) begin
//      void'(randomize(rnd) with {rnd > 0; rnd <= 100;});
//      if(rnd < req_rate_i) begin
//        dut_req_port_o.data_req  = 1'b1;
//        dut_req_port_o.data_size = 2'b11;
//        vaddr = val;
//        // generate linear read
//        `APPL_WAIT_COMB_SIG(clk_i, dut_req_port_i.data_gnt)
//        // increment by set size
//        val = (val + 2 ** DCACHE_INDEX_WIDTH) % (MemWords<<3);
//      end
//      `APPL_WAIT_CYC(clk_i,1)
//      dut_req_port_o.data_req      = '0;
//    end
//    dut_req_port_o.data_req      = '0;
//    dut_req_port_o.data_size     = '0;
//    dut_req_port_o.kill_req      = '0;
//  endtask : genSetSeqRead

//  task automatic genWrapSeq();
//    automatic logic [63:0] val;
//    vaddr                        = CachedAddrBeg;
//    dut_req_port_o.data_req      = '0;
//    dut_req_port_o.data_size     = '0;
//    dut_req_port_o.kill_req      = '0;
//    val                          = '0;
//    while(~seq_end_req) begin
//      dut_req_port_o.data_req  = 1'b1;
//      dut_req_port_o.data_size = 2'b11;
//      vaddr = val;
//      // generate wrapping read of 1 cachelines
//      vaddr = CachedAddrBeg + val;
//      val = (val + 8) % (1*(DCACHE_LINE_WIDTH/64)*8);
//      `APPL_WAIT_COMB_SIG(clk_i, dut_req_port_i.data_gnt)
//      `APPL_WAIT_CYC(clk_i,1)
//    end
//    dut_req_port_o.data_req      = '0;
//    dut_req_port_o.data_size     = '0;
//    dut_req_port_o.kill_req      = '0;
//  endtask : genWrapSeq


///////////////////////////////////////////////////////////////////////////////
// Sequence application
///////////////////////////////////////////////////////////////////////////////

  initial begin : p_stim
    paddr            = '0;
        
    seq_end_ack                  = '0;

    // print some info
    $display("%s> current configuration:", PortName);
    $display("%s> TlbHitRate       %d",    PortName, TlbHitRate);
    $display("%s> RndSeed           %d",   PortName, RndSeed);

    `APPL_WAIT_CYC(clk_i,1)
    `APPL_WAIT_SIG(clk_i,~rst_ni)

    $display("%s> starting application", PortName);
    while(~seq_last_i) begin
        `APPL_WAIT_SIG(clk_i,seq_run_i)
      unique case(seq_type_i)
        RANDOM_SEQ: begin
          $display("%s> start random sequence with %04d responses and req_rate %03d", PortName, seq_num_resp_i, req_rate_i);
          genRandReq();
        end
        LINEAR_SEQ: begin
          $display("%s> start linear sequence with %04d responses and req_rate %03d", PortName, seq_num_resp_i, req_rate_i);
          genSeqRead();
        end
        SET_SEQ: begin
          $display("%s> start set sequence with %04d responses and req_rate %03d", PortName, seq_num_resp_i, req_rate_i);
          genSetSeqRead();
        end
        WRAP_SEQ: begin
          $display("%s> start wrapping sequence with %04d responses and req_rate %03d", PortName, seq_num_resp_i, req_rate_i);
          genWrapSeq();
        end
        IDLE_SEQ: begin
          `APPL_WAIT_SIG(clk_i,seq_end_req)
        end
        BURST_SEQ: begin
          $fatal(1, "Burst sequence not implemented for read port agent");
        end
        CONST_SEQ: begin
          $fatal(1, "Constant sequence not implemented for read port agent.");
        end
      endcase // seq_type_i
      seq_end_ack = 1'b1;
      $display("%s> stop sequence", PortName);
      `APPL_WAIT_CYC(clk_i,1)
      seq_end_ack = 1'b0;
    end
    $display("%s> ending application", PortName);
  end


///////////////////////////////////////////////////////////////////////////////
// Response acquisition
///////////////////////////////////////////////////////////////////////////////
  // we do not check for errors here yet
  initial begin : p_acq
    int    n;
    
    seq_done_o   = 1'b0;
    seq_end_req  = 1'b0;
    prog_end     = 1'b0;
    
    `ACQ_WAIT_CYC(clk_i,1)
    `ACQ_WAIT_SIG(clk_i,~rst_ni)

    ///////////////////////////////////////////////
    // loop over tests
    n=0;
    while(~seq_last_i) begin
      `ACQ_WAIT_SIG(clk_i, seq_run_i)
      seq_done_o = 1'b0;

      $display("%s> %s", PortName, test_name_i);
      for (int k=0;k<seq_num_resp_i && seq_type_i != IDLE_SEQ;k++) begin        
        `ACQ_WAIT_SIG(clk_i, icache_areq_i.fetch_valid | lsu_dtlb_hit_i) 
      end
      
      seq_end_req = 1'b1;
      `ACQ_WAIT_SIG(clk_i, seq_end_ack)
      seq_end_req = 1'b0;

      `ACQ_WAIT_CYC(clk_i,1)
      seq_done_o = 1'b1;
      n++;
    end

    prog_end = 1'b1;
  end

endprogram // tb_csrport

