## instructions

<br>$ cd UEYEcam/ueye/</br>
<br>$ make</br>

<br>$ cd UEYEcam</br>
<br>$ cd capture_frames</br>
<br>$ python execute.py</br>

### NOTE::what's vital to do before acquring frames ....

Before image capturing:
	open ueye_software (i.e. ueye demo)
	open camera
	open settings (-->ueye(tab)-->properties)
	adjust AES/AGC (Exposure/Gain ..)
	adjust Image (Color gains)
	adjust Focus
	-->OK
	File(tab)-->save_parameter--> Parameter Set
	File(tab)-->save_parameter--> to file --> /..*Ueye Root Dir*../parameters/presetParams.ini (UEYEcam/ueye/parameters/presetParams.ini)
	close camera
	exit application


You may adjust some arguments within execute.py.
you ll find the frames captured within the local dir <rgb_input>

## if ueye is not installed:
	# [Ubuntu] run the following script to install it.
		$ cd ueye_install/
		$ bash install_all.sh
	# [Windows/other] https://en.ids-imaging.com/download-details/AB00979.html?os=linux&version=&bus=64&floatcalc=

