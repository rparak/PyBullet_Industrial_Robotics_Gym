def Get_Environment_ID(name: str, env_mode: int) -> str:
    """
    Description:
        Get a string of the gym environment ID for parameters defined by the function input.

    Args:
        (1) name [string]: Name of the robotic structure.
        (2) env_mode [int]: ....

    Returns:
        (1) parameter [string]: The string of the desired gym environment ID.
    """

    try:
        assert env_mode in ['Default', 'Safe']

        return {
            'Universal_Robots_UR3': lambda env_m: f'Ur3-{env_m}-Reach-v0',
            'ABB_IRB_120': lambda env_m: f'AbbIrb120-{env_m}-Reach-v0',
            'ABB_IRB_120_L_Ax': lambda env_m: f'AbbIrb120L-{env_m}-Reach-v0',
            'ABB_IRB_14000_R': lambda env_m: f'AbbIrb14000R-{env_m}-Reach-v0',
            'ABB_IRB_14000_L': lambda env_m: f'AbbIrb14000L-{env_m}-Reach-v0',
            'EPSON_LS3_B401S': lambda env_m: f'EpsonLs3-{env_m}-Reach-v0'
        }[name](env_mode)
    
    except AssertionError as error:
        print(f'[ERROR] Information: {error}')