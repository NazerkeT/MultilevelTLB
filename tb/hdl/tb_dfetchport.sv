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

program tb_dfetchport  import tb_pkg::*; import ariane_pkg::*; #(
  parameter string       PortName      = "$D-Port",
  parameter              TlbHitRate    = 95,
  parameter              StoreRate     = 50,
  parameter              MemWords      = 1024*1024,// in 64bit words
  parameter              RndSeed       = 1110,
  parameter              Verbose       = 0
) (
  input logic           clk_i,
  input logic           rst_ni,

  // to testbench master
  ref   string          test_name_i,
  input  logic [6:0]    req_rate_i, //a rate between 0 and 100 percent
  input  seq_t          seq_type_i,
  input  logic          tlb_rand_en_i,
  input  logic          seq_run_i,
  input  logic [31:0]   seq_num_resp_i,
  input  logic          seq_last_i,
  output logic          seq_done_o,

  // expresp interface
  output [riscv::VLEN-1:0]       exp_vaddr_o,
  input  [riscv::VLEN-1:0]       exp_vaddr_i,
  input  [riscv::VLEN-1:0]       act_vaddr_i,
  input  logic                   mem_valid_i,
  input  logic [riscv::PLEN-1:0] mem_paddr_i,
  input  exception_t             mem_exception_i,

  // DUT-MMU interface
  output  exception_t              misaligned_ex_o,
  output  logic                    lsu_req_o,        // send address translation
  output  logic [riscv::VLEN-1:0]  lsu_vaddr_o,      // send virtual address
  output  logic                    lsu_is_store_o,   // the translation is requested by a store

  // Compare the information below with what is received from TB mem!
  // Cycle 0 
  input logic                      lsu_dtlb_hit_i,   // sent in the same cycle as the request if translation hits in the DTLB
  input logic [riscv::PPNW-1:0]    lsu_dtlb_ppn_i,   // ppn (send same cycle as hit)
  input logic                      all_tlbs_checked_i, // sent in min - the same cycle, max - three cycles   
  // Cycle 1  
  input logic                      lsu_valid_i,      // translation is valid
  input logic [riscv::PLEN-1:0]    lsu_paddr_i,      // translated address
  input exception_t                lsu_exception_i   // address translation threw an exception
);

  // leave this
  timeunit 1ps;
  timeprecision 1ps;

  logic [63:0] vaddr;
  logic seq_end_req, seq_end_ack, prog_end;
    
  // no delay between
  assign exp_vaddr_o = vaddr;
  assign lsu_vaddr_o = vaddr;
  
  logic [riscv::PPNW-1:0] act_paddr;
  assign act_paddr = {lsu_dtlb_ppn_i, act_vaddr_i[11:0]};
  
  // for now, dont play with misaligned exceptions from SU
  assign misaligned_ex_o = '0;
  
///////////////////////////////////////////////////////////////////////////////
// Helper tasks
///////////////////////////////////////////////////////////////////////////////
  
  task automatic commonRoutine();
    automatic logic [63:0] val;
    // generate random store address requests
    void'(randomize(val) with {val > 0; val <= 100;});
    if (val < StoreRate) begin
        lsu_is_store_o = 1'b1;
    end
          
    `APPL_WAIT_COMB_SIG(clk_i, all_tlbs_checked_i)
    if (!lsu_dtlb_hit_i) begin
        lsu_req_o = 1'b1;
        `APPL_WAIT_COMB_SIG(clk_i, all_tlbs_checked_i)
    end
    
  endtask : commonRoutine
  
  task automatic genRandReq();
    automatic logic [63:0] val;

    void'($urandom(RndSeed));

    vaddr          = '0;
    lsu_req_o      = '0;
    lsu_is_store_o = '0;

    while(~seq_end_req) begin
      // randomize request
      lsu_req_o      = '0;
      lsu_is_store_o = '0;
      // generate random control events
      void'(randomize(val) with {val > 0; val <= 100;});
      if(val < req_rate_i) begin
        lsu_req_o = 1'b1;
        // generate random address
        void'(randomize(val) with {val >= 0; val < (MemWords<<3);});
        vaddr = val;
        
        // generate random store address requests
        // and wait for all_tlbs_checked_i
        commonRoutine();
        
      end
      `APPL_WAIT_CYC(clk_i,1)
      
    end

    lsu_req_o      = '0;
    lsu_is_store_o = '0;

  endtask : genRandReq

  task automatic genSeqRead();
    automatic logic [63:0] val;
    vaddr          = '0;
    lsu_req_o      = '0;
    lsu_is_store_o = '0;
    val            = '0;
    while(~seq_end_req) begin
      lsu_req_o  = 1'b1;
      vaddr = val;
      // generate linear read
      val = (val + 8) % (MemWords<<3);
      
      // generate random store address requests
      // and wait for all_tlbs_checked_i
      commonRoutine();
      
      `APPL_WAIT_CYC(clk_i,1) 
    end
    lsu_req_o      = '0;
    lsu_is_store_o = '0;
    
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
    vaddr            = '0;
    lsu_req_o        = '0;
    lsu_is_store_o   = '0;
    
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
  
  initial begin : p_acq
    bit tlb_is_ok, ptw_is_ok;
    progress status;
    string failingTests, tmpstr1, tmpstr2;
    int    n;
    
    status       = new(PortName);
    failingTests = "";
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
      status.reset(seq_num_resp_i);
      for (int k=0;k<seq_num_resp_i && seq_type_i != IDLE_SEQ;k++) begin
        tlb_is_ok = '0;
        ptw_is_ok = '0;
        
        `ACQ_WAIT_SIG(clk_i, all_tlbs_checked_i)
        
        if (lsu_dtlb_hit_i) begin
            // note: wildcard as defined in right operand!
            tlb_is_ok = (mem_valid_i && (mem_paddr_i == act_paddr)) && (exp_vaddr_i == act_vaddr_i);
            ptw_is_ok = 1'b1;
        end else begin
            `ACQ_WAIT_SIG(clk_i, lsu_valid_i)
            ptw_is_ok = (mem_valid_i && (mem_paddr_i == lsu_paddr_i)) && (exp_vaddr_i == act_vaddr_i);
            
            // repeat the check for tlb
            // now at the same time tasks should req 
            `ACQ_WAIT_SIG(clk_i, all_tlbs_checked_i)
            if (lsu_dtlb_hit_i) begin
                tlb_is_ok = (mem_valid_i && (mem_paddr_i == act_paddr)) && (exp_vaddr_i == act_vaddr_i);
            end
        end

        if(Verbose | !tlb_is_ok) begin
          tmpstr1 =  $psprintf("vector: %02d - %06d -- exp_vaddr: %16X -- exp_data: %16X",
                      n, k, exp_vaddr_i, mem_paddr_i);
          tmpstr2 =  $psprintf("vector: %02d - %06d -- act_vaddr: %16X -- act_data: %16X",
                      n, k, act_vaddr_i, act_paddr);
          $display("TLB Issue\n");
          $display("%s> %s", PortName, tmpstr1);
          $display("%s> %s", PortName, tmpstr2);
        end 
        
        if(Verbose | !ptw_is_ok) begin
            tmpstr1 =  $psprintf("vector: %02d - %06d -- exp_vaddr: %16X -- exp_data: %16X",
                    n, k, exp_vaddr_i, mem_paddr_i);
            tmpstr2 =  $psprintf("vector: %02d - %06d -- act_vaddr: %16X -- act_data: %16X",
                        n, k, act_vaddr_i, lsu_paddr_i);
            $display("PTW Issue\n");
            $display("%s> %s", PortName, tmpstr1);
            $display("%s> %s", PortName, tmpstr2);
        end
        
        if (Verbose | lsu_exception_i.valid) begin
            $display("Exception issue, debug more\n");
        end

        if(!tlb_is_ok) begin
            failingTests = $psprintf("TLB Issue: %s%s> %s\n%s> %s\n", failingTests, PortName, tmpstr1, PortName, tmpstr2);
        end
        
        if(!ptw_is_ok) begin
            failingTests = $psprintf("PTW Issue: %s%s> %s\n%s> %s\n", failingTests, PortName, tmpstr1, PortName, tmpstr2);
        end
        
        status.addRes(!tlb_is_ok);
        status.addRes(!ptw_is_ok);
        
        status.print();
      end
      seq_end_req = 1'b1;
      `ACQ_WAIT_SIG(clk_i, seq_end_ack)
      seq_end_req = 1'b0;

      `ACQ_WAIT_CYC(clk_i,1)
      seq_done_o = 1'b1;
      n++;
    end
    ///////////////////////////////////////////////

    status.printToFile({PortName, "_summary.rep"}, 1);

    if(status.totErrCnt == 0) begin
      $display("%s> ----------------------------------------------------------------------", PortName);
      $display("%s> PASSED %0d VECTORS", PortName, status.totAcqCnt);
      $display("%s> ----------------------------------------------------------------------\n", PortName);
    end else begin
      $display("%s> ----------------------------------------------------------------------\n", PortName);
      $display("%s> FAILED %0d OF %0d VECTORS\n", PortName , status.totErrCnt, status.totAcqCnt);
      $display("%s> failing tests:", PortName);
      $display("%s", failingTests);
      $display("%s> ----------------------------------------------------------------------\n", PortName);
    end
    prog_end = 1'b1;
  end

endprogram // tb_dfetchport
