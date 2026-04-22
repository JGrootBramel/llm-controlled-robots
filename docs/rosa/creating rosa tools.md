# Creating @tool Functions

LangChain provides a decorator called @tool that can be used to define the actions that the ROSA agent can take in the ROS environment. These functions should take in the necessary parameters and return a string that describes the action taken.

## Elements of a good @tool function

- **Descriptive Name**: The name of the function should describe the action taken as clearly as possible.
- **Type Annotations**: Use type annotations to specify the types of the parameters and return value.
- **Docstring**: Include a docstring that describes what the function does and the purpose of each parameter. These docstrings will be consumed by the LLM to determine if the tool is relevant to the user query.
- **Parameter Validation**: Validate the parameters to ensure that they are of the correct type and within the expected range.
- **Safety Checks**: Include safety checks in the tool to ensure that the agent does not perform unsafe actions.
- **Error Handling**: Include error handling to gracefully handle any exceptions that may occur during the execution of the tool.
- **Return Value**: The function should return a string that describes either (1) the action taken by the tool and its results, or (2) an error message if the tool fails to execute. You may also choose to return a dict, list, or other object that properly resolves as a string.

## Example

```bash
@tool
def descriptive_tool_name(param1: type1, param2: type2) -> str:
    """
    Description of the tool.
    
    :param param1: Description of param1 and how it is used.
    :param param2: Description of param2 and how it is used.
    """
    # Your code here ...
    return f"Action taken: {ACTION}, retrieved data: {DATA}."
```

Copied from [Github Wiki of ROSA](https://github.com/nasa-jpl/rosa/wiki/Developer-Documentation)

## Worked example: `manipulation.pick_object`

This is how the refactored tools are wired. The whole file lives at
[`src/limo_llm_control/tools/manipulation.py`](../../src/limo_llm_control/tools/manipulation.py).
The robot exposes `std_srvs/Trigger` services — one per skill — and the
ROSA tool is a three-line `ServiceProxy` call with a nice docstring and
graceful error handling.

```python
import rospy
from langchain.tools import tool
from std_srvs.srv import Trigger
from ..ros_clients import ensure_rospy


_PICK_SRV = "/arm_control/pick"


def _trigger(service_name: str) -> str:
    ensure_rospy()
    try:
        rospy.wait_for_service(service_name, timeout=2.0)
    except Exception:
        return f"Service '{service_name}' is unavailable."
    try:
        proxy = rospy.ServiceProxy(service_name, Trigger)
        resp = proxy()
        prefix = "OK" if resp.success else "FAIL"
        return f"{prefix}: {resp.message}"
    except Exception as exc:
        return f"Call to {service_name} failed: {exc}"


@tool
def pick_object() -> str:
    """Close the gripper on the latest target pose.

    Calls ``/arm_control/pick`` (``std_srvs/Trigger``). The arm node uses
    whatever was last published on its ``~target_pose`` topic (by default
    the detector's ``/red_cubes/latest_pose``).
    """
    return _trigger(_PICK_SRV)
```

Why this shape works well for ROSA:

- **One concern per tool.** `pick_object` only picks; it never drives
  the base or runs perception. This lets the LLM sequence the steps
  itself (or use the pre-built `fetch_red_cubes` helper).
- **Thin client, fat node.** All motion / safety logic lives in
  `arm_control_node.py` on the robot. The remote tool is just a
  ServiceProxy + a docstring.
- **Stable string contract.** The tool always returns a human-readable
  `"OK: …"` / `"FAIL: …"` string — easy for the LLM to reason over and
  easy to unit-test with `unittest.mock`.
- **No new message types.** We only use `std_srvs/Trigger`,
  `std_srvs/SetBool`, and `geometry_msgs/PoseStamped`. Adding a new
  skill is "add one Trigger service" — no `.srv` compilation required.

See `tests/unit/test_rosa_tools_clients.py` for how we unit-test these
tools without any ROS installation.
