General Responsibilities:
- All members must be able to operate Marvin v1 independently and competently.

# TODO
### Akshart
- Uncomment and tune staged placeholder camera mounts (`urdf/*`) and EKF inputs (`odom2`/`odom3`) when extra cameras are installed
- Integrate and tune new camera sources in `ekf_fusion.yaml`
- Validate/tune all sensor frame transforms against measured hardware
- Runtime validation of wheel joint-state source (`/joint_states` vs `/wheel_ticks` path)
- indicator for emag
    sshpass -p 'RASunsw' ssh -tt -o StrictHostKeyChecking=no marvin_v2@192.168.1.80

### Ben
- Finish mapping software
- Data throughput monitoring

### Pat
- Sort out teensy code for science and arm

### Pratham
- integrate arm ui into gui
- Buttons to open different payloads

### Jackson
- learn teensy code