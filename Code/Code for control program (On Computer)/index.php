<?php
	$endp1=8;						# End point 1
	$endp2=$endp1*2;					# End point 2
	$filetowrite='field.txt';	# Path to file for writing
	$intrachkpdist=20;					# Distance between two checkpoints
	
	$shelltocompilenruncpp = './compile_run.sh';
?>
<html>
	<head>
		<title>The Digging Bot: User Interface</title>
	</head>
<body>
<div align='center'>

	<h2 style='color:#800000;'>The Digging Bot's User Interface</h2>
	<br>
	<form method='post' action='#'>
	<table width="500" border='0' cellpadding='0' cellspacing='0'>
	<tr>
		<td width='10'><img src='images/checkpoint.jpeg' width='50' height='50' /></td>
		<td style="font-size:30px;"> <img src='images/hline.jpeg' width='100%' height='50' /> </td>
		<td width='10'><img src='images/checkpoint.jpeg' width='50' height='50' /></td>
		<td style='font-size:30px;'> <img src='images/hline.jpeg' width='100%' height='50' /> </td>
		<td width='10'><img src='images/checkpoint.jpeg' width='50' height='50' /></td>
	</tr>
<?php


	if( isset($_POST['action']) )
	{
		$count=1;

		$fh = fopen($filetowrite, 'w');
		fwrite($fh,$endp1." ".$endp2."\n");
		//echo $endp1." ".$endp2."<br>";

		$fh = fopen($filetowrite, 'a');
		if( isset($_POST['hidcount']) )
		{
			$count = 1;
			while( $count <= $_POST['hidcount'] )
			{
				if( isset($_POST['dia'.$count]) )
				{
					if( strlen($_POST['dia'.$count]) > 0 )
					fwrite($fh,($count*$intrachkpdist)." ".$_POST['dia'.$count]."\n");
				}
				$count++;	
			}	
			$count = 1;
			$reduceby=2;
			while( $count <= $_POST['hidcount'] )
			{
				if( isset($_POST['dia'.($count + ($endp2-$reduceby))]) )
				{
					if( strlen($_POST['dia'.($count + ($endp2-$reduceby))]) > 0 )
					fwrite($fh,(($count + ($endp2-$reduceby))*$intrachkpdist)." ".$_POST['dia'.($count + ($endp2-$reduceby))]."\n");
				}
				$count++;	
				$reduceby+=2;
			}	

	
			echo "Ready to run with field plan ....";
			#############################################
			########### Executing the shell script ######	
			#############################################
			echo exec($shelltocompilenruncpp.' > temp.dump');
			echo system('./a.out');			
			exec('chmod 777 '.$filetowrite);

		}	
	}


	$count=1;
	$reduceby=2;
	while( $count < $endp1 )
	{
	?>
		<tr>
			<td style='font-size:30px;' align='middle' valign='bottom'> <img src='images/vline.jpeg' width='100%' height='24'  /> </td>
			<td style="font-size:30px;"><img src='images/track.jpeg' width='100%' height='24' /></td>
			<td width='10' style='font-size:30px;' align='middle'><img src='images/vline.jpeg' width='100%' height='24' /></td>
			<td style="font-size:30px;"><img src='images/track.jpeg' width='100%' height='24' /></td>
			<td style='font-size:30px;' align='middle' valign='bottom'> <img src='images/vline.jpeg' width='100%' height='24' /> </td>
		</tr>

		<tr>
			<td  align='middle' valign='bottom'> 
				<div  style='background-image:url("images/checkpointbg.jpeg");' >
					<select name='dia<?php echo $count + ($endp2-$reduceby); $reduceby+=2; ?>' >
						<option value=''>?</option>
						<option value='a'>2</option>
						<option value='b'>3</option>
						<option value='c'>4</option>
						<option value='d'>5</option>
						<option value='e'>6</option>
					</select> 
						
				</div>								
			</td>

			<td style="font-size:30px;"><img src='images/track.jpeg' width='100%' height='24' /></td>
			<td width='10' style='font-size:30px;' align='middle'><img src='images/vline.jpeg' width='100%' height='24' /></td>
			<td style="font-size:30px;"><img src='images/track.jpeg' width='100%' height='24' /></td>
			<td  align='middle' valign='bottom'> 
					<select name='dia<?php echo $count; ?>'>
						<option value=''>?</option>						
						<option value='a'>2</option>
						<option value='b'>3</option>
						<option value='c'>4</option>
						<option value='d'>5</option>
						<option value='e'>6</option>
					</select> 							
			</td>
		</tr>


		<tr>
			<td style='font-size:30px;' align='middle' valign='bottom'> <img src='images/vline.jpeg' width='100%' height='24'  /> </td>
			<td style="font-size:30px;"><img src='images/track.jpeg' width='100%' height='24' /></td>
			<td width='10' style='font-size:30px;' align='middle'><img src='images/vline.jpeg' width='100%' height='24' /></td>
			<td style="font-size:30px;"><img src='images/track.jpeg' width='100%' height='24' /></td>	
			<td style='font-size:30px;' align='middle' valign='bottom'> <img src='images/vline.jpeg' width='100%' height='24' /> </td>
		</tr>






	<?php
		$count++;
	}
?>
	<tr>
		<td width='10'><img src='images/checkpoint.jpeg' width='50' height='50' /></td>
		<td style="font-size:30px;"> <img src='images/hline.jpeg' width='100%' height='50' /> </td>
		<td width='10'><img src='images/checkpoint.jpeg' width='50' height='50' /></td>
		<td style='font-size:30px;'> <img src='images/hline.jpeg' width='100%' height='50' /> </td>
		<td width='10'><img src='images/checkpoint.jpeg' width='50' height='50' /></td>
	</tr>
	<input type='hidden' name='hidcount' value='<?php echo $count; ?>' />
	<tr style='background-color:#CCC;'>
		<td colspan="3" align='center'><input type="submit" name="action" value='Run' /></td>
		<td colspan="2" align='center'><input type="reset" value="Reset" /></td>
	</tr>
	</table>
	
			

	</form>
</div>
</body>
</html>
