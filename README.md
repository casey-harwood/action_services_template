# action_services_template

Minimal ROS 2 template for action + service communication.

This workspace is intentionally lightweight so students can start from a clear MWE and extend it with their own interfaces, node names, and logic.

## Purpose

This template is designed to be a clean starting point:
- no artificial sleep delays
- no verbose node logging
- action feedback/result and service responses preserved

Students are expected to replace names, interfaces, and logic with their own project-specific implementations.

## Included Packages

### `interface_templates`
- Defines example custom interfaces:
	- `action/IncrementToGoal.action`
	- `srv/IncrementNumber.srv`

### `node_templates`
- Provides example nodes:
	- `service_server_node.py`
	- `action_server_node.py`
	- `action_client_node.py`
- Provides launch file:
	- `launch/template_demo.launch.py`

## What This Template Demonstrates

- A service server handling request/response
- An action server handling goal/feedback/result
- An action client sending a goal and receiving feedback/result
- Async service calls from inside an action execute callback

## Build

If these packages are not already in your ROS 2 workspace:

```bash
mkdir -p <workspace>/src
cp -r interface_templates node_templates <workspace>/src/
```

```bash
cd <your_ros2_workspace>
colcon build --symlink-install
source install/setup.bash
```

## Run

### Launch all nodes

```bash
ros2 launch node_templates template_demo.launch.py
```

### Or run each node manually

Terminal 1:
```bash
source <your_ros2_workspace>/install/setup.bash
ros2 run node_templates service_server_node
```

Terminal 2:
```bash
source <your_ros2_workspace>/install/setup.bash
ros2 run node_templates action_server_node
```

Terminal 3:
```bash
source <your_ros2_workspace>/install/setup.bash
ros2 run node_templates action_client_node
```

### Direct CLI tests

Action call:
```bash
ros2 action send_goal /increment_to_goal interface_templates/action/IncrementToGoal "{target: 5}"
```

Service call:
```bash
ros2 service call /increment_number interface_templates/srv/IncrementNumber "{number: 10}"
```

## Runtime Flow

1. Service server starts and offers `/increment_number`
2. Action server waits for service, then offers `/increment_to_goal`
3. Action client sends a goal
4. Action server loops by calling service and publishing feedback
5. Action server returns final result when target is reached

## Using STUDENT TODO Blocks

Search the codebase for `STUDENT TODO` to find the required customization points.

These blocks identify where to:
- rename interfaces and ROS entity names
- replace request/response and goal/feedback/result fields
- swap demo callback logic for project-specific behavior
- update launch entries after package/executable renaming

## Student Customization Checklist

Start by searching for `STUDENT TODO` blocks in the template files. These mark the exact locations where students should rename interfaces, update node/action/service names, and replace demo logic.

Update these in your own project version:

1. Package names in `package.xml`, `setup.py`, and launch files
2. Interface names/fields in `.action` and `.srv` files
3. Node names and topic/service/action names in each node
4. Goal validation logic in action server `goal_callback`
5. Service callback computation in `increment_callback`
6. Action client goal construction in `send_goal`

## Student Customization Guide

### 1) Rename for your project
- package names (`interface_templates`, `node_templates`)
- executable names in `setup.py`
- node/action/service names in code and launch file

### 2) Replace interfaces
- create your own `.srv` and `.action` definitions
- update imports in all nodes
- update request/response and goal/feedback/result fields in callbacks

### 3) Replace behavior
- update service callback computation logic
- update action goal validation rules
- update action execute loop and completion criteria
- update client goal construction

### 4) Keep this structure
- constructor setup for ROS entities
- callback-based execution model
- action feedback + result flow

## Recommended Student Steps

1. Duplicate this template into a new package pair for your assignment.
2. Rename interfaces first, then make code compile again.
3. Replace service logic.
4. Replace action execution logic.
5. Validate via CLI action/service calls.

## Learning Objectives

- Distinguish action workflows from service workflows
- Use async calls safely in ROS 2 Python nodes
- Implement goal validation, feedback, and final result semantics
- Build a reusable node/template baseline for larger systems

## Notes

- This template removes artificial delays and verbose logging.
- Feedback/result behavior is kept so end-to-end action flow remains visible.
- Constructor instructional comments are intentionally left in place for teaching.
