
# EDIT THIS TO DEFINE THE AUTOFOCUS VALUES
AUTOFOCUS_RANGES={78:[330,360],39:[372,390],135:[310,325],68:[335,360]}


def calculate_Automatic_focus(camera_height):
    
    try:
        af=AUTOFOCUS_RANGES[int(camera_height)]
    except:
        print(f"\n\n\t\tHeight {camera_height} has not been defined.... Declare and define within AUTOFOCUS_RANGES data structure\n\n")
    autofocus_min, autofocus_max=af[0],af[1]

    return autofocus_min,autofocus_max