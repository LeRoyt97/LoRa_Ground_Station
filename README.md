Modified version of the Ground-Tracking-Station software created by Mathew Clutter (2021) for use with Montana Space Grant Consortium's BOREALIS high altitude ballooning program. The program was modified to be compatible with a LoRa-only balloon configuration, and does not require use of the IRIDIUM satellite service.







### In-progress:

* Bug/Crash fixes
* Create executable file
* Merge Branches into master once in functional states (Balloon\_Commands, and gps\_extraction\_and\_storage).





### Link to GitHub:

https://github.com/LeRoyt97/LoRa\_Ground\_Station/tree/master







### Graphical User Interface (GUI):

PyQt5 was used to create the GUI: 

&nbsp;	https://pypi.org/project/PyQt5/



PyQt5 Designer is a useful tool that allows for drag and drop of various widgets, containers, text boxes, etc. in a more visually friendly environment. The Designer.exe application can be installed via pip install:

 	pip install pyqt5-tools



File location confusion:

&nbsp;	The Designer.exe file can be hard to find after finishing the pip install. I found it by navigating to the following directory and then searching for "designer.exe":



&nbsp;	C:\\Users\\%username%\\AppData\\Local\\Programs\\Python\\Python39\\Lib\\site-packages

&nbsp;			^					^

&nbsp;			| Your Username				| Python version; yours may be different.



&nbsp;	



