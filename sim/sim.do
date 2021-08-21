# Following commands is a good start for basic simulation case, 
# but does not represent full scenario.
# Feel free to test more conditions

# set up clock and reset
add_force {/mmu/clk_i} -radix hex {1 0ns} {0 50000ps} -repeat_every 100000ps
add_force {/mmu/rst_ni} -radix hex {0 0ns}
run 100 ns
run 100 ns
add_force {/mmu/rst_ni} -radix hex {1 0ns}
run 100 ns
run 100 ns
add_force {/mmu/enable_translation_i} -radix hex {1 0ns}
add_force {/mmu/en_ld_st_translation_i} -radix hex {1 0ns}
add_force {/mmu/flush_i} -radix hex {0 0ns}
add_force {/mmu/flush_tlb_i} -radix hex {0 0ns}
run 100 ns
add_force {/mmu/misaligned_ex_i.valid} -radix hex {0 0ns}
add_force {/mmu/misaligned_ex_i.tval} -radix hex {0 0ns}
add_force {/mmu/misaligned_ex_i.cause} -radix hex {0 0ns}
run 100 ns
add_force {/mmu/sum_i} -radix hex {0 0ns}
add_force {/mmu/mxr_i} -radix hex {1 0ns}
add_force {/mmu/satp_ppn_i} -radix hex {1 0ns}
add_force {/mmu/asid_i} -radix hex {1 0ns}
add_force {/mmu/ld_st_priv_lvl_i} -radix hex {1 0ns}
add_force {/mmu/priv_lvl_i} -radix hex {1 0ns}
run 100 ns
add_force {/mmu/lsu_req_i} -radix hex {0 0ns}
add_force {/mmu/lsu_is_store_i} -radix hex {0 0ns}
run 100 ns
 
# simulate 4K page in L1-ITLB 
add_force {/i_itlb/tags_q[2].asid} -radix hex {1 0ns}
add_force {/i_itlb/tags_q[2].valid} -radix hex {1 0ns}
add_force {/i_itlb/tags_q[2].is_2M} -radix hex {0 0ns}
add_force {/i_itlb/tags_q[2].is_1G} -radix hex {0 0ns}
add_force {/i_itlb/tags_q[2].vpn2} -radix hex {0cc 0ns}
add_force {/i_itlb/tags_q[2].vpn1} -radix hex {0aa 0ns}
add_force {/i_itlb/tags_q[2].vpn0} -radix hex {0ff 0ns}
add_force {/i_itlb/content_q[2].ppn} -radix hex {ffffffffff 0ns}
run 100 ns
add_force {/mmu/icache_areq_i.fetch_req} -radix hex {1 0ns}
add_force {/mmu/icache_areq_i.fetch_vaddr} -radix bin {011001100010101010011111111000000000000 0ns}
run 100 ns
add_force {/i_itlb/tags_q[2].valid} -radix hex {0 0ns}
run 100 ns
 
# simulate 4K page in L2-TLB(put under specific ind for this page size) 
add_force {/i_tlb_l2/tags_q[7][1].asid} -radix hex {1 0ns}
add_force {/i_tlb_l2/tags_q[7][1].valid} -radix hex {1 0ns}
add_force {/i_tlb_l2/tags_q[7][1].is_2M} -radix hex {0 0ns}
add_force {/i_tlb_l2/tags_q[7][1].is_1G} -radix hex {0 0ns}
add_force {/i_tlb_l2/tags_q[7][1].vpn2} -radix hex {0cc 0ns}
add_force {/i_tlb_l2/tags_q[7][1].vpn1} -radix hex {0aa 0ns}
add_force {/i_tlb_l2/tags_q[7][1].vpn0} -radix hex {0ff 0ns} 
add_force {/i_tlb_l2/content_q[7][1].ppn} -radix hex {ffffffffff 0ns}
run 100 ns

# simulate ptw walk upon req
add_force {/mmu/icache_areq_i.fetch_req} -radix hex {1 0ns}
run 100 ns
add_force {/mmu/i_ptw/allow_access} -radix hex {1 0ns}
run 100 ns
run 100 ns
run 100 ns
add_force {/mmu/req_port_i.data_gnt} -radix hex {1 0ns}
run 100 ns
add_force {/mmu/req_port_i.data_rvalid} -radix hex {1 0ns}
add_force {/mmu/req_port_i.data_gnt} -radix hex {0 0ns}
run 100 ns
run 100 ns
run 100 ns
add_force {/mmu/req_port_i.data_rvalid} -radix hex {0 0ns}
add_force {/mmu/req_port_i.data_rvalid} -radix hex {1 0ns}
add_force {/mmu/req_port_i.data_gnt} -radix hex {1 0ns}
run 100 ns
run 100 ns
add_force {/mmu/req_port_i.data_gnt} -radix hex {0 0ns}
run 100 ns
add_force {/mmu/req_port_i.data_gnt} -radix hex {1 0ns}
add_force {/mmu/req_port_i.data_rdata} -radix hex {bc3c3c3ccf 0ns}
run 100 ns
run 100 ns

# simulate 4K page in L1-DTLB 
add_force {/i_dtlb/tags_q[1].asid} -radix hex {1 0ns}
add_force {/i_dtlb/tags_q[1].valid} -radix hex {1 0ns}
add_force {/i_dtlb/tags_q[1].is_2M} -radix hex {0 0ns}
add_force {/i_dtlb/tags_q[1].is_1G} -radix hex {0 0ns}
add_force {/i_dtlb/tags_q[1].vpn2} -radix hex {0dd 0ns}
add_force {/i_dtlb/tags_q[1].vpn1} -radix hex {0ee 0ns}
add_force {/i_dtlb/tags_q[1].vpn0} -radix hex {0bb 0ns}
add_force {/i_dtlb/content_q[1].ppn} -radix hex {aaaaaaaaaa 0ns}
run 100 ns
add_force {/mmu/lsu_req_i} -radix hex {1 0ns}
add_force {/mmu/lsu_vaddr_i} -radix bin {011011101011101110010111011000000000000 0ns}
run 100 ns

# simulate 4K page in L2-TLB 
add_force {/i_tlb_l2/tags_q[3][0].asid} -radix hex {1 0ns}
add_force {/i_tlb_l2/tags_q[3][0].valid} -radix hex {1 0ns}
add_force {/i_tlb_l2/tags_q[3][0].is_2M} -radix hex {0 0ns}
add_force {/i_tlb_l2/tags_q[3][0].is_1G} -radix hex {0 0ns}
add_force {/i_tlb_l2/tags_q[3][0].vpn2} -radix hex {0dd 0ns}
add_force {/i_tlb_l2/tags_q[3][0].vpn1} -radix hex {0ee 0ns}
add_force {/i_tlb_l2/tags_q[3][0].vpn0} -radix hex {0bb 0ns}
add_force {/i_tlb_l2/content_q[3][0].ppn} -radix hex {aaaaaaaaaa 0ns}
run 100 ns

add_force {/mmu/lsu_req_i} -radix hex {1 0ns}
add_force {/mmu/lsu_vaddr_i} -radix bin {011011101011101110010111011000000000000 0ns}
run 100 ns

# simulate 1G page in L2-TLB 
add_force {/i_tlb_l2/tags_q[3][0].valid} -radix hex {0 0ns}

add_force {/i_tlb_l2/tags_q[5][0].asid} -radix hex {1 0ns}
add_force {/i_tlb_l2/tags_q[5][0].valid} -radix hex {1 0ns}
add_force {/i_tlb_l2/tags_q[5][0].is_2M} -radix hex {0 0ns}
add_force {/i_tlb_l2/tags_q[5][0].is_1G} -radix hex {0 0ns}
add_force {/i_tlb_l2/tags_q[5][0].vpn2} -radix hex {0dd 0ns}
add_force {/i_tlb_l2/tags_q[5][0].vpn1} -radix hex {0ee 0ns}
add_force {/i_tlb_l2/tags_q[5][0].vpn0} -radix hex {0bb 0ns}
add_force {/i_tlb_l2/content_q[5][0].ppn} -radix hex {aaaaaaaaaa 0ns}
run 100 ns

add_force {/i_tlb_l2/tags_q[3][0].valid} -radix hex {1 0ns}
