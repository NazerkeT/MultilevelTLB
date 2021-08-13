# set up clock and reset
add_force {/tlb_l2/clk_i} -radix hex {1 0ns} {0 50000ps} -repeat_every 100000ps
add_force {/tlb_l2/rst_ni} -radix hex {0 0ns}
run 100 ns
run 100 ns
add_force {/tlb_l2/rst_ni} -radix hex {1 0ns}
run 100 ns

# simulate 4K page
add_force {/tlb_l2/tags_q[7][0].asid} -radix hex {1 0ns}
add_force {/tlb_l2/tags_q[7][0].valid} -radix hex {1 0ns}
add_force {/tlb_l2/tags_q[7][0].is_2M} -radix hex {0 0ns}
add_force {/tlb_l2/tags_q[7][0].is_1G} -radix hex {0 0ns}
add_force {/tlb_l2/tags_q[7][0].vpn2} -radix hex {0cc 0ns}
add_force {/tlb_l2/tags_q[7][0].vpn1} -radix hex {0aa 0ns}
add_force {/tlb_l2/tags_q[7][0].vpn0} -radix hex {0ff 0ns}
add_force {/tlb_l2/content_q[7][0].ppn} -radix hex {ffffffffff 0ns}
run 100 ns
add_force {/tlb_l2/lu_asid_i} -radix hex {1 0ns}
add_force {/tlb_l2/lu_vaddr_i} -radix bin {011001100010101010011111111000000000000 0ns}
add_force {/tlb_l2/lu_access_i} -radix hex {1 0ns}
run 100 ns
add_force {/tlb_l2/lu_access_i} -radix hex {0 0ns}
add_force {/tlb_l2/tags_q[7][0].valid} -radix hex {0 0ns}
run 100 ns

# simulate 2M page
add_force {/tlb_l2/tags_q[2][1].asid} -radix hex {1 0ns}
add_force {/tlb_l2/tags_q[2][1].valid} -radix hex {1 0ns}
add_force {/tlb_l2/tags_q[2][1].is_2M} -radix hex {1 0ns}
add_force {/tlb_l2/tags_q[2][1].is_1G} -radix hex {0 0ns}
add_force {/tlb_l2/tags_q[2][1].vpn2} -radix hex {0cc 0ns}
add_force {/tlb_l2/tags_q[2][1].vpn1} -radix hex {0aa 0ns}
add_force {/tlb_l2/tags_q[2][1].vpn0} -radix hex {0ff 0ns}
add_force {/tlb_l2/content_q[2][1].ppn} -radix hex {aaaaaaaaaa 0ns}
run 100 ns
add_force {/tlb_l2/lu_asid_i} -radix hex {1 0ns}
add_force {/tlb_l2/lu_vaddr_i} -radix bin {011001100010101010011111111000000000000 0ns}
add_force {/tlb_l2/lu_access_i} -radix hex {1 0ns}
run 100 ns
add_force {/tlb_l2/lu_access_i} -radix hex {0 0ns}
run 100 ns
run 100 ns

# simulate 1G page
add_force {/tlb_l2/tags_q[2][1].valid} -radix hex {0 0ns}
add_force {/tlb_l2/tags_q[4][1].asid} -radix hex {1 0ns}
add_force {/tlb_l2/tags_q[4][1].valid} -radix hex {1 0ns}
add_force {/tlb_l2/tags_q[4][1].is_2M} -radix hex {0 0ns}
add_force {/tlb_l2/tags_q[4][1].is_1G} -radix hex {1 0ns}
add_force {/tlb_l2/tags_q[4][1].vpn2} -radix hex {0cc 0ns}
add_force {/tlb_l2/tags_q[4][1].vpn1} -radix hex {0aa 0ns}
add_force {/tlb_l2/tags_q[4][1].vpn0} -radix hex {0ff 0ns}
add_force {/tlb_l2/content_q[4][1].ppn} -radix hex {cccccccccc 0ns}
run 100 ns
add_force {/tlb_l2/lu_asid_i} -radix hex {1 0ns}
add_force {/tlb_l2/lu_vaddr_i} -radix bin {011001100010101010011111111000000000000 0ns}
add_force {/tlb_l2/lu_access_i} -radix hex {1 0ns}
run 100 ns
add_force {/tlb_l2/lu_access_i} -radix hex {0 0ns}
run 100 ns
run 100 ns

add_force {/tlb_l2/tags_q[4][1].valid} -radix hex {0 0ns}
run 100 ns

# simulate update logic
add_force {/tlb_l2/update_i.valid} -radix hex {1 0ns}
add_force {/tlb_l2/update_i.is_1G} -radix hex {1 0ns}
add_force {/tlb_l2/update_i.is_2M} -radix hex {0 0ns}
add_force {/tlb_l2/update_i.asid} -radix hex {1 0ns}
add_force {/tlb_l2/update_i.content.ppn} -radix hex {cccccccccc 0ns}
add_force {/tlb_l2/update_i.vpn} -radix bin {011001100010101010011111111 0ns}
run 100 ns

