import sys
import os
import time
import numpy as np

# Add project root to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import forcedimension_core.dhd as dhd
import forcedimension_core.drd as drd

def main():
    print("Initializing Force Dimension device...")
    
    # Initialize DRD (Robotic SDK)
    if drd.openID(0) < 0:
        print(f"Error: drd.openID failed: {dhd.os_independent.kbGet()}")
        return

    # Check if supported
    # In python drd wrapper we might not have isSupported but we have isInitialized

    # Auto init
    if not drd.isInitialized(0):
        print("Initializing device (this may move the device)...")
        if drd.autoInit(0) < 0:
            print("Error: drd.autoInit failed")
            return
    
    if drd.start(0) < 0:
        print("Error: drd.start failed")
        return

    # Move to center first (using DRD regulation)
    print("Moving to center...")
    # Since we don't have drdMoveTo in python wrapper yet, we can try to rely on manual spring force or implement drdMoveTo binding
    # But user wants "Spring centered" behavior which is manual force calculation usually.
    # The C++ example uses drdMoveTo to go to center, then drdStop(true) to keep forces on? No, drdStop(true) leaves forces enabled? 
    # Actually drdStop(true) usually disables regulation but keeps gravity compensation.
    
    # Let's switch to manual force control loop for "Spring Centering"
    # First stop regulation but leave gravity comp if possible? 
    # drd.stop(True, 0) # True means leave forces on (gravity comp usually)
    drd.stop(True, 0)
    
    print("Device ready. Centering Spring Mode Enabled.")
    print("Press Ctrl+C to exit.")
    
    center_pos = np.zeros(3)
    current_pos = np.zeros(3)
    current_rot = np.eye(3)
    
    # Spring stiffness (N/m) and Torsional stiffness (Nm/rad)
    K_spring = 200.0 
    K_torsion = 0.5 # Try 0.5 Nm/rad
    
    # Store initial rotation to compute delta if needed, or just use identity as target
    # For auto-centering, target is always identity rotation relative to base.
    
    prev_rot = np.eye(3)
    dhd.getPositionAndOrientationFrame(center_pos, prev_rot, 0)
    
    while True:
        # 1. Read position and rotation matrix
        dhd.getPositionAndOrientationFrame(current_pos, current_rot, 0)
        
        # Action (Deviation from center) in Device Frame
        action_device = current_pos.copy()
        
        # Action Mapping for Position
        action_pos_mapped = np.zeros(3)
        action_pos_mapped[0] = -action_device[1]
        action_pos_mapped[1] = action_device[0]
        action_pos_mapped[2] = action_device[2]
        
        # 3. Calculate Spring Force (Linear)
        force = -K_spring * action_device
        
        # 4. Calculate Spring Torque (Angular)
        from scipy.spatial.transform import Rotation as R
        
        # Compute delta euler similar to forcedimension_expert.py
        # R_diff = R_curr * R_prev^T
        R_curr = current_rot
        R_prev = prev_rot
        R_diff = R_curr @ np.linalg.inv(R_prev)
        
        # In forcedimension_expert, this delta is used for velocity control.
        # In autocenter, we want to center to absolute zero (identity matrix).
        # So we treat "prev_rot" as "target_rot" (Identity) conceptually for torque calc?
        # No, torque needs absolute deviation from identity.
        
        # Let's calculate delta_euler exactly as in expert for display/check
        delta_euler = R.from_matrix(R_diff).as_euler('xyz')
        
        # Update prev_rot for next iteration (to behave like expert's delta)
        prev_rot = current_rot.copy()
        
        # Mapping rotation vector (delta) to match your expert mapping
        action_rot_mapped = np.zeros(3)
        action_rot_mapped[0] = -delta_euler[0]
        action_rot_mapped[1] = -delta_euler[1]
        action_rot_mapped[2] = delta_euler[2]
        
        # For TORQUE calculation (Autocentering), we still need ABSOLUTE deviation from Identity
        r_abs = R.from_matrix(current_rot)
        rot_vec_abs = r_abs.as_rotvec() 
        torque = -K_torsion * rot_vec_abs

        # Safety clamp for force and torque
        force_mag = np.linalg.norm(force)
        if force_mag > 10.0:
            force = force * (10.0 / force_mag)
            
        torque_mag = np.linalg.norm(torque)
        if torque_mag > 0.5: # 0.5 Nm max
            torque = torque * (0.5 / torque_mag)
        
        # 5. Apply Force and Torque
        dhd.setForceAndTorqueAndGripperForce(force[0], force[1], force[2], torque[0], torque[1], torque[2], 0.0, 0)
        
        # Print status occasionally
        if int(time.time() * 20) % 20 == 0:
                # Format: Action(Mapped Delta) [x y z rx ry rz] | F | T
                full_action = np.concatenate([action_pos_mapped, action_rot_mapped])
                print(f"Action(MapDelta): {np.round(full_action, 3)} | F: {np.round(force, 1)} | T: {np.round(torque, 2)}", end='\r')
        
        time.sleep(0.001) # 1kHz loop is ideal for haptics

if __name__ == "__main__":
    main()
