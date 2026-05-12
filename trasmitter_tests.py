def no_ack_response_test(pkt, Tests_State):
    # Hypothèse : le champ ack est à un mais la prochaine réponse n'est pas un ack.
    # Résultat attendu : Le message actuel se répète un certain nombre de fois.
    print("[P] Running no_ack test...")

    if pkt.wpan.ack_request != None and pkt.wpan.ack_request.int_value == False:
        Tests_State["no_ack_test_tx"] = True

def ack_response_test(pkt, Tests_State):
    # Hypothèse : le champ ack est à un et la prochaine réponse est un ack.
    # Résultat attendu : Comportement normal.
    print("[P] Running ack_response test...")

    if pkt.wpan.ack_request != None and pkt.wpan.ack_request.int_value == True:
        Tests_State["ack_test_tx"] = True

def no_frame_pending_test(pkt, Tests_State):
    # Hypothèse : le champ frame_pending est à 0.
    # Résultat attendu : On trouve le champ frame_pending à 0
    print("[P] Running no_frame_pending test...")
    

    if pkt.wpan.pending != None and pkt.wpan.pending.int_value == False:
        Tests_State["no_frame_pending_test_tx"] = True

def frame_pending_test(pkt, Tests_State):
    # Hypothèse : le champ frame_pending est à 1.
    # Résultat attendu : On trouve le champ frame_pending à 1
    print("[P] Running frame_pending test...")

    if pkt.wpan.pending != None and pkt.wpan.pending.int_value == True:
        Tests_State["frame_pending_test_tx"] = True

def short_addr_test(pkt, Tests_State):
    # Hypothèse : le champ short_addr est à 1.
    # Résultat attendu : On trouve le champ short_addr à 1
    print("[P] Running short_addr test...")

    src = (pkt.wpan.src_addr_mode != None and pkt.wpan.src_addr_mode.int_value == 2)
    dst = (pkt.wpan.dst_addr_mode != None and pkt.wpan.dst_addr_mode.int_value == 2)

    if src and dst:
        Tests_State["short_addr"] = True

def long_addr_test(pkt, Tests_State):
    # Hypothèse : le champ long_addr est à 1.
    # Résultat attendu : On trouve le champ long_addr à 1
    print("[P] Running long_addr test...")

    src = (pkt.wpan.src_addr_mode != None and pkt.wpan.src_addr_mode.int_value == 3)
    dst = (pkt.wpan.dst_addr_mode != None and pkt.wpan.dst_addr_mode.int_value == 3)

    if src and dst:
        Tests_State["long_addr"] = True

def no_pan_id_compression_test(pkt, Tests_State):
    # Hypothèse : le champ pan_id_compression est à 0.
    # Résultat attendu : On trouve le champ pan_id_compression à 0
    print("[P] Running no_pan_id_compression test...")

    if pkt.wpan.pan_id_compression != None and pkt.wpan.pan_id_compression.int_value == False:
        Tests_State["no_pan_id_compression"] = True

def pan_id_compression_test(pkt, Tests_State):
    # Hypothèse : le champ pan_id_compression est à 1.
    # Résultat attendu : On trouve le champ pan_id_compression à 1
    print("[P] Running pan_id_compression test...")

    if pkt.wpan.pan_id_compression and pkt.wpan.pan_id_compression.int_value == True:
        Tests_State["pan_id_compression"] = True

def security_enabled_test(pkt, Tests_State):
    # Hypothèse : le champ security_enabled est à 1.
    # Résultat attendu : On trouve le champ security_enabled à 1
    print("[P] Running security_enabled test...")

    if pkt.wpan.security and pkt.wpan.security.int_value == True:
        Tests_State["security_enabled"] = True

def security_disabled_test(pkt, Tests_State):
    # Hypothèse : le champ security_enabled est à 0.
    # Résultat attendu : On trouve le champ security_enabled à 0
    print("[P] Running security_disabled test...")

    if pkt.wpan.security != None and pkt.wpan.security.int_value == False:
        Tests_State["security_disabled"] = True

def data_frame_type_test(pkt, Tests_State):
    # Hypothèse : le champ FrameType est de type FrameType::Data
    # Résultat attendu : On trouve le champ FrameType à 1
    print("[P] Running data_frame_type test...")
    
    if pkt.wpan.frame_type != None and pkt.wpan.frame_type.int_value == 1:
        Tests_State["data_frame_type"] = True 

def beacon_frame_type_test(pkt, Tests_State):
    # Hypothèse : le champ FrameType est de type FrameType::Beacon
    # Résultat attendu : On trouve le champ FrameType à 0
    print("[P] Running beacon_frame_type test...")
    
    if pkt.wpan.frame_type != None and pkt.wpan.frame_type.int_value == 0:
        Tests_State["beacon_frame_type"] = True 

def ack_frame_type_test(pkt, Tests_State):
    # Hypothèse : le champ FrameType est de type FrameType::Ack
    # Résultat attendu : On trouve le champ FrameType à 2
    print("[P] Running ack_frame_type test...")
    
    if pkt.wpan.frame_type != None and pkt.wpan.frame_type.int_value == 2:
        Tests_State["ack_frame_type"] = True 

def mac_cmd_frame_type_test(pkt, Tests_State):
    # Hypothèse : le champ FrameType est de type FrameType::Data
    # Résultat attendu : On trouve le champ FrameType à 1
    print("[P] Running mac_cmd_frame_type test...")
    
    if pkt.wpan.frame_type != None and pkt.wpan.frame_type.int_value == 3:
        Tests_State["mac_cmd_frame_type"] = True

def multipurpose_frame_type_test(pkt, Tests_State):
    # Hypothèse : le champ FrameType est de type FrameType::Multipurpose
    # Résultat attendu : On trouve le champ FrameType à 5
    print("[P] Running multipurpose_frame_type test...")
    
    if pkt.wpan.frame_type != None and pkt.wpan.frame_type.int_value == 5:
        Tests_State["multipurpose_frame_type"] = True

def fragment_frame_type_test(pkt, Tests_State):
    # Hypothèse : le champ FrameType est de type FrameType::Fragment
    # Résultat attendu : On trouve le champ FrameType à 6
    print("[P] Running fragment_frame_type test...")
    
    if pkt.wpan.frame_type != None and pkt.wpan.frame_type.int_value == 6:
        Tests_State["fragment_frame_type"] = True

def extended_frame_type_test(pkt, Tests_State):
    # Hypothèse : le champ FrameType est de type FrameType::Extended
    # Résultat attendu : On trouve le champ FrameType à 7
    print("[P] Running data_frame_type test...")
    
    if pkt.wpan.frame_type != None and pkt.wpan.frame_type.int_value == 7:
        Tests_State["extended_frame_type"] = True    
