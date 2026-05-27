# next_gen_nav2_traffic


# InnerNavigationServices Workflow Diagram

This document describes the workflow in `InnerNavigationServices`, specifically the `await_and_send_goal` workflow, implemented using Crossflow.

## Workflow Diagram

```mermaid
graph TD
    Start([scope.start]) --> AwaitReq[Await Requests]
    
    AwaitReq -->|stream| CheckGoal[Check Goal]
    
    CheckGoal -->|Ok| CancelGoal[Cancel Goal]
    CancelGoal -->|Ok| TrimNode[Trim Downstream]
    TrimNode --> RequestGoal[Request Goal]
    
    CheckGoal -->|Err| RequestGoal
    
    RequestGoal -->|Ok| UpdateGoal[Update Goal Client]
    UpdateGoal --> MonitorGoal[Monitor Navigation]
    MonitorGoal --> RetryNav[Retry Navigation]

    RetryNav -->|Retry| RequestGoal
    RetryNav -->|Err| CleanupGoal[Cleanup Goal Client]
    
    AwaitReq -->|output| Terminate([scope.terminate])
```

## Node Descriptions

- **`await_new_requests`**: A continuous service that listens for `InnerNavigationTarget` events and streams `InnerNavigationRequest` objects.
- **`check_existing_goal`**: Checks if the agent already has an active goal. If so, it proceeds to cancel it; otherwise, it requests a new goal.
- **`async_cancel_goal`**: An asynchronous service that cancels the existing Nav2 goal.
- **`Trim Downstream`**: If the cancellation is successful, this node trims downstream operations to avoid conflicts before requesting a new goal.
- **`async_request_new_goal`**: An asynchronous service that sends a new `NavigateToPose` goal to Nav2.
- **`update_goal_client`**: Updates the `InnerNavigationClient` component with the new goal handle.
- **`async_monitor_ongoing_navigation`**: Monitors the progress of the navigation goal, handling feedback and final results (Succeeded, Aborted, Cancelled).
- **`retry_navigation`**: A map block that decides whether to retry the navigation if it was aborted.
- **`cleanup_goal_client`**: Cleans up the goal client state in the component upon completion or failure.


# NavigationServices Workflow Diagram

This document describes the workflow in `NavigationServices`, implemented using Crossflow.

## Workflow Diagram

```mermaid
graph TD
    Start([scope.start]) --> ForkClone{Fork Clone}
    
    ForkClone --> MonitorClients[Monitor Clients]
    ForkClone --> MonitorFeedback[Monitor Feedback]
    
    MonitorFeedback -- streams --> FeedbackBuffer[(Feedback Buffer)]
    
    MonitorClients -- streams --> BufferAccess[Buffer Access]
    FeedbackBuffer -.-> BufferAccess
    
    BufferAccess --> PublishFeedback[Publish Feedback]
    
    MonitorClients -- output --> Terminate([scope.terminate])
```

## Node Descriptions

- **`monitor_inner_navigation_clients`**: A continuous service that manages incoming navigation requests and responds to orders when the entire navigation is completed.
- **`monitor_inner_navigation_feedback`**: A continuous service that monitors feedback from inner navigation clients.
- **`FeedbackBuffer`**: A buffer that keeps the last 10 `InnerNavigationFeedback` items.
- **`Buffer Access`**: Accesses the `FeedbackBuffer` to retrieve feedback for requests coming from `monitor_inner_navigation_clients`.
- **`publish_navigation_feedback`**: A service that publishes the navigation feedback.
