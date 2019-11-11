#!/usr/bin/python

import sys
import os.path
import re

if( len( sys.argv ) < 3 ):
	print("Must supply a filename to read from and a filename to write to on the command line.")
else:
	fileName = sys.argv[1]
	if( not os.path.isfile( fileName ) ):
		print("You must supply a valid file.")
	
	else:
		start = None
		end = None
		if( len( sys.argv ) == 5 ):
			start = int( sys.argv[3] )
			end = int( sys.argv[4] )
			
		print( "\n\nConverting file " + fileName + "\n" )
		f = open( fileName, 'r' );
		fileFormatCorrect = False;

		if( '<?xml version="1.0"?>' in f.readline() ):
			if( '<opencv_storage>' in f.readline() ):
				if( '<SimplePointTracking>' in f.readline() ):
					fileFormatCorrect = True;

		if( fileFormatCorrect ):
			#outFileName = fileName.rstrip(".xml") + ".csv"
			outFileName = sys.argv[2]
			print( "Writing new file to " + outFileName + "\n")
			outputFile = open( outFileName, 'w+' )
			row = "Original Filename," + fileName;
			outputFile.write( row + ",\n" );
			row = "ModifiedNumTimesteps," + str( int(end) - int(start) + 1 );
			outputFile.write( row + ",\n" );
			inTimestep = False;
			tagEmbeddedInData = False;
			writeData = False;
			endOfFileReached = False;
			rowData = []
			for line in f:
				if( '</SimplePointTracking>' in line ):
					endOfFileReached = True;
					line = line.replace('</SimplePointTracking>','')
				if( '<Timestep' in line and '</Timestep' in line ):
					matches = re.findall( '<(Timestep[0-9]*)>', line )
					if( len( matches ) != 1 ):
						print( "Error parsing empty timestep." )
						break;
					timestepValue = matches[0].replace('Timestep',"").replace(" ","")
					if( start <= int(timestepValue) <= end ):
						outputFile.write( timestepValue + "," + "\n")
				else:
					if( '<Timestep' in line ):
						inTimestep = True
						rowData = []
						rowData.append( line.replace('<Timestep',"").replace('>',"").replace('\n','').replace(" ","") )
						continue;

					if( '</Timestep' in line ):
						inTimestep = False
						if( line[:10] != '</Timestep' ):
							tagEmbeddedInData = True

					if( inTimestep ):
						nums = line.split()
						for num in nums:
							rowData.append( num )
					else:
						
						if( tagEmbeddedInData ):
							nums = re.sub( '</Timestep[0-9]*>', '', line )
							nums = nums.split()
							for num in nums:
								rowData.append( num )
							tagEmbeddedInData = False
							writeData = True

						else:
							#simply read row and parse out single row info	
							matches = re.findall( '<([a-zA-Z]*)>(.*)</([a-zA-Z]*)>', line )
							if( len( matches ) == 1 and len( matches[0] ) == 3 ):
								outputFile.write( matches[0][0] + "," + matches[0][1] + ",\n")

							else:
								print( "Error parsing line. Expected paired tags in a single line, instead found: \n" + line )
								print( "With matches:\n" )
								for m in matches:
									print( matches )
									print("\n")
								break;

					#Now write everything out
					if( len( rowData ) and writeData ):
						if( start <= int(rowData[0]) <= end ):
							row = rowData[0] + ","
							for i in range( 1, len(rowData), 3 ):
								row += rowData[i] + "|" + rowData[i+1] + "|" + rowData[i+2] + ","

							outputFile.write( row + "\n" );
						writeData = False;
						rowData = []
				if( endOfFileReached ):
					break;

			outputFile.close()
		else:
			print( "\nFile does not have correct format\n" )
		f.close()