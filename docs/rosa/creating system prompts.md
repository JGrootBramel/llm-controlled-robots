# Adding System Prompts

System prompts are used to guide ROSA's behavior and provide context for its actions. The RobotSystemPrompts class contains attributes that describe the robot's identity, environment, and objectives. These prompts are used by the LLM to generate responses that are consistent with the robot's persona and capabilities.

## Attributes of the `RobotSystemPrompts` class

The RobotSystemPrompts class contains the following attributes, each of which provides context for the ROSA agent:

- `embodiment_and_persona`: Gives the agent a sense of identity and helps it understand its role.
- `about_your_operators`: Provides information about the operators who interact with the robot, which can help the agent understand the context of the interaction.
- `critical_instructions`: Provides critical instructions that the agent should follow to ensure the safety and well-being of the robot and its operators.
- `constraints_and_guardrails`: Gives the robot a sense of its limitations and informs its decision-making process.
    about_your_environment: Provides information about the physical and digital environment in which the robot operates.
- `about_your_capabilities`: Describes what the robot can and cannot do, which can help the agent understand its limitations.
- `nuance_and_assumptions`: Provides information about the nuances and assumptions that the agent should consider when interacting with the robot.
- `mission_and_objectives`: Describes the mission and objectives of the robot, which can help the agent understand its purpose and goals.
- `environment_variables`: Provides information about the environment variables that the agent should consider when interacting with the robot. e.g. $ROS_MASTER_URI, or $ROS_IP.

## Elements of good system prompts

- **Use a consistent tone**: Use a consistent tone and style across all prompts to maintain the robot's persona.
- **Be clear and concise**: Keep the prompts clear and concise to ensure that the agent can understand and act on them.
- **Avoid jargon**: Avoid technical jargon and use plain language to describe the robot's capabilities and limitations.
- **Provide contextual information**: Include contextual information that helps the agent understand its role and objectives.
- **Do not contradict yourself**: Ensure that the prompts are consistent with each other and do not contradict the robot's identity or capabilities.
- **Update prompts as needed**: Update the prompts as needed to reflect changes in the robot's capabilities or environment.

## Example

```bash
prompts = RobotSystemPrompts(
    embodiment_and_persona="You are a cool robot that does cool stuff.",
    critical_instructions="You must confirm all actions with the operator before proceeding. Failure to do so might result in damage to the robot or its environment.",
)
```


Copied from [Github Wiki of ROSA](https://github.com/nasa-jpl/rosa/wiki/Developer-Documentation)