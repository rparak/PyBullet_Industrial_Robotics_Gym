def Get_Environment_ID(name: str, mode: int) -> str:
    """
    Description:
        Get a string of the gym environment ID for parameters defined by the function input.

    Args:
        (1) name [string]: Name of the robotic structure.
        (2) mode [int]: ....

    Returns:
        (1) parameter [string]: The string of the desired gym environment ID.
    """

    try:
        assert mode in ['Default', 'Safe']

        return {
            'Universal_Robots_UR3': lambda x: f'Ur3-{x}-Reach-v0',
            'ABB_IRB_120': lambda x: f'AbbIrb120-{x}-Reach-v0',
            'ABB_IRB_120_L_Ax': lambda x: f'AbbIrb120L-{x}-Reach-v0',
            'ABB_IRB_14000_R': lambda x: f'AbbIrb14000R-{x}-Reach-v0',
            'ABB_IRB_14000_L': lambda x: f'AbbIrb14000L-{x}-Reach-v0',
            'EPSON_LS3_B401S': lambda x: f'EpsonLs3-{x}-Reach-v0'
        }[name](mode)
    
    except AssertionError as error:
        print(f'[ERROR] Information: {error}')