MCP Server installtion and running!

Step 1: Pull the Official Image
Open your terminal and pull the pre-built image directly from the repository.

docker pull ghcr.io/ajtudela/nav2_mcp_server:latest


Step 2: Run the Docker container with --net=host.
Note: --net=host is critical. Without it, the Docker container is isolated and cannot see your robot's ROS topics.

docker run -it --rm --net=host ghcr.io/ajtudela/nav2_mcp_server:latest


Curser installtion:
https://cursor.com/download
Note: Click Download for linux and install through software installer.

# Cursor MCP Setup for Nav2

This guide fixes the configuration structure error and ensures Cursor can access your ROS 2 environment.

### 1. Fix the Configuration File

Your previous error occurred because the configuration was missing the required root object.

1. In Cursor, go to **Settings > Features > MCP**.
2. If you see an error, click the **Open JSON** link next to it.
3. **Replace** the entire file content with this valid JSON structure:

```json
{
  "mcpServers": {
    "nav2_mcp_server": {
      "command": "/home/darshan/.local/bin/uv",
      "args": [
        "run",
        "--directory",
        "/home/darshan/mcp_course/nav2_mcp_server",
        "nav2_mcp_server"
      ],
      "env": {
        "ROS_DOMAIN_ID": "0",
        "ROS_LOCALHOST_ONLY": "1"
      }
    }
  }
}

```

*Note: Ensure the path to `uv` matches your system (check with `which uv`).*

### 2. Launch Cursor correctly (Crucial!)

You cannot launch Cursor from the desktop icon because it won't load the ROS 2 environment variables. You must launch it from a terminal.

1. Close Cursor completely.
2. Open a terminal and run:
```bash
# 1. Load ROS 2 Humble
source /opt/ros/humble/setup.bash

# 2. Launch Cursor in this environment
cursor .

```



### 3. Verification

1. Open the **Cursor Settings > Features > MCP** panel.
2. You should see a **Green Light** 🟢 next to `nav2_mcp_server`.
3. Open the **AI Chat** (`Ctrl+L`) and type:
> "List the currently active ROS 2 nodes."



If the AI responds with a list of nodes (like `/collision_monitor`, `/velocity_smoother`), the connection is successful.




Read this Gemini file: for running docker

https://gemini.google.com/app/ed4ca1df7980759f




Am I connected to ROS2 System ?

What are ros2 nodes, topics, services running on the system ?

What is the localisation node we are using ?

What is the global planner we are using ?

What is the type of robot we are uisng in simulation ?

What is the weather in Mysure ?

Find the nearest obstacle in front of robot using laserscan data ?