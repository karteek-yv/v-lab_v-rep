<?php
$myfile = fopen("newfile1.txt", "w") or die("Unable to open file!");
$txt = $_POST["exp_no"];
fwrite($myfile, $txt."\n");
$txt = $_POST["x_co"];
fwrite($myfile, $txt."\n");
$txt = $_POST["y_co"];
fwrite($myfile, $txt."\n");
$txt = $_POST["z_co"];
fwrite($myfile, $txt);
fclose($myfile);
//echo nl2br ("\r Please wait \r \r");
sleep(2);
$myfile = fopen("thetas.txt", "r") or die("Unable to open file!");
while ($line = fgets($myfile)) {
  // <... Do your work with the line ...>
	echo nl2br ("\n");
	echo($line);
}
fclose($myfile);
$myfile = fopen("thetas.txt", "w") or die("Unable to open file!");
fwrite($myfile," Upload failed \n Please try again \n \n If the problem persists, please contact admin.\n");
fclose($myfile);
//header("Refresh: 5; URL=http://localhost:8000/tp_intl.html");
exit();
?> 
