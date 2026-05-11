def no_ack_response_test(pkt, Tests_State):
    # Hypothèse : le champ ack est à un mais la prochaine réponse n'est pas un ack.
    # Résultat attendu : Le message actuel se répète un certain nombre de fois.
    print("[P] Running no_ack test...")

    if pkt['wpan']['wpan_wpan_ack_request'] != None and pkt['wpan']['wpan_wpan_ack_request'] == False:
        Tests_State["no_ack_test_tx"] = True

def ack_response_test(pkt, Tests_State):
    # Hypothèse : le champ ack est à un et la prochaine réponse est un ack.
    # Résultat attendu : Comportement normal.
    print("[P] Running ack_response test...")

    if pkt['wpan']['wpan_wpan_ack_request'] and pkt['wpan']['wpan_wpan_ack_request'] == True:
        Tests_State["ack_test_tx"] = True

def no_frame_pending_test(pkt, Tests_State):
    # Hypothèse : le champ frame_pending est à 0.
    # Résultat attendu : On trouve le champ frame_pending à 0
    print("[P] Running no_frame_pending test...")

    if pkt['wpan']['wpan_wpan_pending'] != None and pkt['wpan']['wpan_wpan_pending'] == False:
        Tests_State["no_frame_pending_test_tx"] = True

def frame_pending_test(pkt, Tests_State):
    # Hypothèse : le champ frame_pending est à 1.
    # Résultat attendu : On trouve le champ frame_pending à 1
    print("[P] Running frame_pending test...")

    if pkt['wpan']['wpan_wpan_pending'] and pkt['wpan']['wpan_wpan_pending'] == True:
        Tests_State["frame_pending_test_tx"] = True

def short_addr_test(pkt, Tests_State):
    # Hypothèse : le champ short_addr est à 1.
    # Résultat attendu : On trouve le champ short_addr à 1
    print("[P] Running short_addr test...")

    src = (pkt['wpan']['wpan_wpan_src_addr_mode'] != None and pkt['wpan']['wpan_wpan_src_addr_mode'] == '2')
    dst = (pkt['wpan']['wpan_wpan_dst_addr_mode'] != None and pkt['wpan']['wpan_wpan_dst_addr_mode'] == '2')

    if src and dst:
        Tests_State["short_addr"] = True

def long_addr_test(pkt, Tests_State):
    # Hypothèse : le champ long_addr est à 1.
    # Résultat attendu : On trouve le champ long_addr à 1
    print("[P] Running long_addr test...")

    src = (pkt['wpan']['wpan_wpan_src_addr_mode'] != None and pkt['wpan']['wpan_wpan_src_addr_mode'] == '3')
    dst = (pkt['wpan']['wpan_wpan_dst_addr_mode'] != None and pkt['wpan']['wpan_wpan_dst_addr_mode'] == '3')

    if src and dst:
        Tests_State["long_addr"] = True