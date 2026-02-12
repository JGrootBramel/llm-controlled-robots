## Quick Start

1. **Install Docker** (if not already installed):
   ```bash
   sudo apt update && sudo apt install -y docker.io
   sudo systemctl start docker
   sudo usermod -aG docker $USER
   # Log out and log back in
   ```

2. **Run the setup script**:
   ```bash
   ./setup.sh
   ```

3. **Start the simulation**:
   ```bash
   ./run_simulation.sh
   ```

4. **Run the ROSA natural-language controller (inside the container)**:
   - Open a new terminal (keep the simulation terminal running)
   - Find the container id: `sudo docker ps`
   - Enter the container: `sudo docker exec -it <CONTAINER_ID> /bin/bash`
   - Run:
     ```bash
     bash /workspace/llm-pick-me-bots/start_rosa.sh
     ```

**Windows note**: Use WSL2 (recommended). See the Windows section in `docs/ROSA_INTEGRATION.md`.