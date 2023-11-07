from v4l2py.device import Device

def get_camera_id(name_contains:str, port_range:int=10):
    """
    Find the first dev/ port number (id) of a usb camera whose card
    info contains the provided string.

    Args:
        name_contains (str, optional): identifying string in camera's v4l2 card info. Defaults to "USB".
        range (int): number of ports to check. Function will check ports 0, 1, 2... up to this number.

    Returns:
        int: id number (port number i guess)
    """
    cam_cards = {}
    match_id = None
    # loop through possible ids, starting at 0
    for id in range(port_range + 1):
        cam = Device.from_id(id)
        try:
            cam.open()
            cam_cards[id] = cam.info.card
            if name_contains in cam.info.card.lower() and not match_id:
                match_id = id
        except:
            pass
    
    print(f'searching for camera whose name contains "{name_contains}"')
    print("    available cameras:")
    for id, card in cam_cards.items():
        print(f"        id {id}:", card)
    
    if match_id is not None:
        print(f"search success:\n    id {match_id}: {cam_cards[match_id]}")
        return match_id
    else: # no match found
        raise ValueError(f'no camera found with card info containing "{name_contains}"')

id = get_camera_id("integrate")
print(id)