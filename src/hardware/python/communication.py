from dataclasses import dataclass
import struct


observed_dt_data = []
sense_timing_data = []
timeout_data = []
event_sense_data = []
event_ctrl_data = []
event_act_data = []
event_telem_data = [] 

advanced_telemetry_data = {
    'tick_count': [],
    'ctrl_delay_us': []
}

# -----------------------------
# Data model
# -----------------------------
@dataclass
class Sample:
    tick_count: int
    t: float   
    x1: float
    x2: float
    x3: float
    x4: float
    u: float   

t0_ref = {"ts0": None} 

# packet tags
PKT_TELEM  = 1
PKT_EVENTS = 2
PKT_OBSERVED_DT = 3
PKT_CMD = 4
PKT_DEBUG = 5

_telem_struct  = struct.Struct('<Ifffff')   # ts_ms, x1,x2,x3,x4,u  (24 bytes)
_events_struct = struct.Struct('<HHHH')     # event timings         (8 bytes)


def receive(link, t0_ref) -> Sample | None:
    """
    Returns Sample for telemetry packets, None otherwise.
    Prints all packets to terminal for debugging.
    """
    if not link.available():
        return None
    
    try:
        # Get message ID (first byte)
        msg_id = link.rx_obj(obj_type='B', start_pos=0)
        
        if msg_id == PKT_TELEM:
            
            # Read telemetry data: uint32 + 5 floats = 24 bytes
            pos = 1
            tick_count = link.rx_obj(obj_type='I', start_pos=pos); pos += 4
            timestamp_ms = link.rx_obj(obj_type='I', start_pos=pos); pos += 4
            x1 = link.rx_obj(obj_type='f', start_pos=pos); pos += 4
            x2 = link.rx_obj(obj_type='f', start_pos=pos); pos += 4  
            x3 = link.rx_obj(obj_type='f', start_pos=pos); pos += 4
            x4 = link.rx_obj(obj_type='f', start_pos=pos); pos += 4
            u = link.rx_obj(obj_type='f', start_pos=pos)
            
            # Handle time base: milliseconds → seconds, with wraparound handling
            if t0_ref["ts0"] is None:
                t0_ref["ts0"] = timestamp_ms
            
            dt_ms = (timestamp_ms - t0_ref["ts0"]) & 0xFFFFFFFF
            t_seconds = dt_ms / 1000.0
            
            # Print telemetry data
            # print(f"[TELEM] Received telemetry packet")
            # print(f"  tick_count={tick_count}, t={t_seconds:.3f}s, x1={x1:.4f}, x2={x2:.4f}, x3={x3:.4f}, x4={x4:.4f}, u={u:.4f}")
            
            # Return Sample object for plotting/control
            return Sample(
                tick_count=tick_count,
                t=float(t_seconds),
                x1=float(x1),
                x2=float(x2), 
                x3=float(x3),
                x4=float(x4),
                u=float(u)
            )
            
        elif msg_id == PKT_EVENTS:
            
            # Read event timing data: 4 uint32s = 16 bytes  
            pos = 1
            tick_count_telemetry = link.rx_obj(obj_type='I', start_pos=pos); pos += 4
            sense_us = link.rx_obj(obj_type='I', start_pos=pos); pos += 4
            ctrl_us = link.rx_obj(obj_type='I', start_pos=pos); pos += 4
            act_us = link.rx_obj(obj_type='I', start_pos=pos); pos += 4
            telem_us = link.rx_obj(obj_type='I', start_pos=pos)
            
            sense_timing_data.append(sense_us)

            # print(f"[EVENTS] Received events packet")
            # print(f"  sense={sense_us}μs, ctrl={ctrl_us}μs, act={act_us}μs, telem={telem_us}μs")
            
            event_sense_data.append(sense_us)
            # event_ctrl_data.append(ctrl_us-sense_us)
            event_ctrl_data.append(ctrl_us)
            # event_act_data.append(act_us-ctrl_us)
            event_act_data.append(act_us)
            event_telem_data.append(telem_us)

            advanced_telemetry_data['tick_count'].append(tick_count_telemetry)
            advanced_telemetry_data['ctrl_delay_us'].append(ctrl_us-sense_us)

            # Return None for events (not used for control)
            return None
        
        elif msg_id == PKT_OBSERVED_DT:
            # print(f"[PKT_OBSERVED_DT] ")
            
            pos = 1
            observed_dt = link.rx_obj(obj_type='I', start_pos=pos)
            observed_dt_ms = observed_dt/1000
            # print(f"  observed_dt={observed_dt_ms}ms")
            observed_dt_data.append(observed_dt_ms)
            
            # Return None for events (not used for control)
            return None
        
        elif msg_id == PKT_DEBUG:
            
            # Read event timing data: 4 uint16s = 8 bytes  
            pos = 1
            debug_message = link.rx_obj(obj_type='I', start_pos=pos)

            if debug_message == 888:
                # no timout
                timeout_data.append(0)
            elif (debug_message == 999):
                # timeout
                timeout_data.append(1)
            elif (debug_message == 777):
                print("[DEBUG] Entered infinite while loop because angle got too large.")
            else:
                print(f"[PKT_DEBUG] ")
                print(f"  debug message={debug_message}")

            
            # Return None for events (not used for control)
            return None
            
        else:
            print(f"[DEBUG] Unknown packet ID: {msg_id}")
            return None
            
    except Exception as e:
        print(f"[RECEIVE] Error: {e}")
        return None


def send_control(link, u: float, tick_count: int) -> None:
    try:
        # print(f"[DEBUG] sending PKT_CMD={PKT_CMD}, tick={tick_count}, u={u}")
        
        n = 0
        n = link.tx_obj(PKT_CMD, start_pos=n)
        # print(f"  After PKT_CMD: n={n}")
        
        n = link.tx_obj(int(tick_count), start_pos=n)
        # print(f"  After tick_count: n={n}")
        
        n = link.tx_obj(u, start_pos=n)
        # print(f"  After u: n={n}, total bytes={n}")
        
        link.send(n)
        # print(f"[SEND] tick={tick_count} u={u}")
    except Exception as e:
        print(f"[SEND] Error sending control: {e}")
