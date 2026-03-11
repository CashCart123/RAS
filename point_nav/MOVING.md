# Moving `point_nav` Using Git Only

Transfer the package to another PC using Git with two workflows: VS Code → VS Code, and VS Code → CLI Git. Both PCs run Ubuntu 22.04 + ROS 2 Humble.

## Assumptions
- Target PC has ROS 2 Humble installed (`/opt/ros/humble`).
- You keep source in one folder and build in a separate workspace:
  - Source repo: `~/RAS-main`
  - Build workspace: `~/point_nav_ws`
  - Link: `~/point_nav_ws/src/point_nav -> ~/RAS-main/point_nav`

## VS Code → VS Code
On the source PC (publish):
1) Open VS Code in `/home/cashcart/Documents/RAS`.
2) Source Control view → “Initialize Repository” (if not already).
3) Ensure `.gitignore` excludes `build/`, `install/`, `log/`, `__pycache__/`, `*.pyc`.
4) Stage changes → Commit with a message.
5) Click “Publish to GitHub” (or add remote and push):
   - `git branch -M main`
   - `git remote add origin <YOUR_REMOTE_URL>`
   - `git push -u origin main`

On the target PC (clone in VS Code):
1) Install Git: `sudo apt install -y git`.
2) VS Code → Ctrl+Shift+P → “Git: Clone” → paste `<YOUR_REMOTE_URL>`.
3) Choose folder `~` and clone as `RAS-main` (or clone then rename).
4) Open `~/RAS-main` in VS Code.
5) In a terminal, create the build workspace symlink:
   ```bash
   bash ~/RAS-main/point_nav/tools/setup_ws.sh
   ```
6) Build and source in a terminal:
   ```bash
   cd ~/point_nav_ws
   source /opt/ros/humble/setup.bash
   colcon build --packages-select point_nav
   . install/setup.bash
   ```
7) Run as usual:
   ```bash
   ros2 launch point_nav point_nav.launch.py
   ```

## VS Code → CLI Git
On the source PC (publish from VS Code):
1) Same steps as above to commit and push from VS Code.

On the target PC (clone via terminal):
```bash
sudo apt install -y git
cd ~
git clone <YOUR_REMOTE_URL> RAS-main
bash ~/RAS-main/point_nav/tools/setup_ws.sh
cd ~/point_nav_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select point_nav
. install/setup.bash
ros2 launch point_nav point_nav.launch.py
```

## Verify & Troubleshoot
- Package visible: `ros2 pkg list | grep point_nav`
- Node running: `ros2 node list` (look for `/point_nav`)
- Topics: `ros2 topic list | grep -E "goal_point|joy|pose"`
- Different pose topic? Remap at launch:
  ```bash
  ros2 launch point_nav point_nav.launch.py --ros-args -r /zed/zed_node/pose:=/your/pose/topic
  ```
- Another `/joy` publisher active (e.g., joystick driver)? Stop it to avoid conflicts.
