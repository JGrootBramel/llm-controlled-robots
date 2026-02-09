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
