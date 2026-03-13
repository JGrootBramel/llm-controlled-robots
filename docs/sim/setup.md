## Quick Start

1. **Install Docker** (if not already installed):
   ```bash
   sudo apt update && sudo apt install -y docker.io
   sudo systemctl start docker
   sudo usermod -aG docker $USER
   # Log out and log back in
   ```

2. **Docker Compose (robot + remote)** — recommended for testing the two-machine setup:
   - From repo root:
     ```bash
     cp .example.env .env
     # Edit .env and set OPENAI_API_KEY
     docker compose -f sim/docker/docker-compose.yml up --build
     ```
   - The **robot** container runs Gazebo + limo_cobot_sim + limo_rosa_bridge.
   - The **remote** container runs the ROSA app (`python -m limo_llm_control.main`) and connects to the robot via `ROS_MASTER_URI=http://robot:11311`.

3. **Single-container (legacy)**:
   - Run the setup script: `./setup.sh`
   - Start the simulation: `./run_simulation.sh`
   - In a new terminal, exec into the container and run:
     ```bash
     bash /workspace/llm-controlled-robots/start_rosa.sh
     ```

**Windows note**: Use WSL2 (recommended). See the Windows section in `docs/ROSA_INTEGRATION.md`.