# Template Packages Quick Reference


## Package Structure

Use workspace search for `STUDENT TODO` to jump directly to all student customization points.

### interface_templates/
```
interface_templates/
├── CMakeLists.txt
├── package.xml
├── action/
│   └── IncrementToGoal.action
└── srv/
    └── IncrementNumber.srv
```

**IncrementToGoal.action**
- Goal: `uint16 target` - Target number to reach
- Result: `bool success`, `uint16 final_value` - Whether goal was achieved and final value
- Feedback: `uint16 current_value`, `string status` - Progress updates

**IncrementNumber.srv**
- Request: `uint16 number` - Number to increment
- Response: `uint16 incremented_value` - Returns number + 1

### node_templates/
```
node_templates/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── node_templates
├── launch/
│   └── template_demo.launch.py
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── node_templates/
    ├── __init__.py
    ├── service_server_node.py
    ├── action_server_node.py
    └── action_client_node.py
```

## Node Descriptions

### service_server_node.py
**Purpose**: Provides the `/increment_number` service

**What it does**:
- Receives a number
- Returns number + 1
- Returns a response immediately

**Student edits**:
- Replace service name and interface import
- Replace callback math/logic for your use case

### action_server_node.py
**Purpose**: Provides the `/increment_to_goal` action

**What it does**:
- Waits for service to be available
- Validates goals in `goal_callback()`
- Starts incrementing from 1
- Calls increment service repeatedly
- Publishes feedback after each increment
- Succeeds when current value equals target

**Key features for students**:
- Goal validation in `goal_callback()`
- Async service calls in `execute_callback()`
- Feedback publishing
- Cancel handling

**Student edits**:
- Replace action/service names and interface imports
- Replace execute loop, success condition, and feedback content

### action_client_node.py
**Purpose**: Sends a goal to the action server

**What it does**:
- Waits for action server availability
- Sends a goal from user input
- Handles feedback callback
- Handles final result callback
- Shuts down after completion

**Student edits**:
- Replace action name/interface import
- Replace goal fields and input handling to match your action
- Add your own output/logging style as needed

## Execution Timeline (example target=3)

```
Time    Node                  Action
----    ----                  ------
0s      service_server        Starts, ready
0s      action_server         Starts, waits for service
0s      action_client         Starts, waits for action server
~0s     action_client         Sends goal (target=3)
~0s     action_server         Accepts goal, starts execution
~0s     action_server         Calls service (number=1)
~0s     service_server        Receives request (1)
~0s     service_server        Returns response (2)
~0s     action_server         Publishes feedback (current=2, target=3)
~0s     action_server         Calls service (number=2)
~0s     service_server        Receives request (2)
~0s     service_server        Returns response (3)
~0s     action_server         Publishes feedback (current=3, target=3)
~0s     action_server         Returns success
~0s     action_client         Receives result, shuts down
```

Total execution time depends on system scheduling and target size.

## Message Flow Diagram

```
action_client_node          action_server_node          service_server_node
       |                            |                            |
       |----[Goal: target=3]------->|                            |
       |                            |                            |
       |                            |---[Request: number=1]----->|
       |                            |<--[Response: value=2]------|
       |<---[Feedback: current=2]---|                            |
       |                            |---[Request: number=2]----->|
       |                            |<--[Response: value=3]------|
       |<---[Feedback: current=3]---|                            |
       |                      [goal reached]                     |
       |<---[Result: success]-------|                            |
       |                            |                            |
```

## Testing Commands

### Check if nodes are running
```bash
ros2 node list
```

### Monitor topics
```bash
ros2 topic list
ros2 topic echo /increment_to_goal/_action/feedback
```

### Monitor services
```bash
ros2 service list
ros2 service type /increment_number
```

### Monitor actions
```bash
ros2 action list
ros2 action info /increment_to_goal
```

### Send custom goal
```bash
ros2 action send_goal /increment_to_goal interface_templates/action/IncrementToGoal "{target: 5}"
```

### Call service directly
```bash
ros2 service call /increment_number interface_templates/srv/IncrementNumber "{number: 42}"
```

## Educational Points

1. **Service vs Action**: 
   - Service: Quick, synchronous request-response
   - Action: Long-running, asynchronous with feedback and cancellation

2. **Callbacks**:
   - Service: Single callback for each request
   - Action: Multiple callbacks (goal, cancel, execute, feedback)

3. **Async operations**:
   - Service calls from action server use `await`
   - Demonstrates async/await pattern in ROS 2

4. **Logging best practices**:
   - Add only the logging needed for your assignment/debugging
   - Keep logs focused on events students must verify

5. **Timing**:
   - This template intentionally has no artificial sleeps
   - Add timing only if your assignment specifically requires it

## Common Modifications for Exercises

1. **Change target value**: Edit action_client_node.py line with `send_goal(target=3)`
2. **Reject certain goals**: Modify goal_callback in action_server_node.py
3. **Different increment**: Change service to add different values
4. **Countdown instead**: Make it decrement instead of increment
5. **Multiple simultaneous clients**: Run action_client_node multiple times
6. **Add timeout**: Make action fail if it takes too long
7. **Random failures**: Add chance of service failure to test error handling

## Student Rename Map (Recommended)

- `IncrementNumber.srv` -> `<YourService>.srv`
- `IncrementToGoal.action` -> `<YourAction>.action`
- `/increment_number` -> `/<your_service_name>`
- `/increment_to_goal` -> `/<your_action_name>`
- `service_server_node` -> `<your_service_node_name>`
- `action_server_node` -> `<your_action_server_name>`
- `action_client_node` -> `<your_action_client_name>`
