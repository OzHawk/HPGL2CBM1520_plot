10 PRINT CHR$(147)
20 PRINT "-= PLOTTER ART =-"
30 PRINT "THIS PROGRAM WILL CREATE A RANDOM"
40 PRINT "ARTWORK BASED ON THE PARAMETERS"
50 PRINT "ENTERED BY THE USER."
60 PL=PEEK(<???>):IF PL>0 AND PL<5 GOTO 200
60 PRINT "SELECT PLOTTER:"
70 PRINT "[1] COMMODORE 1520"
80 PRINT "[2] HP 7470A"
90 PRINT "[3] DXY-980"
100 PRINT "[4] DPX-3300"
110 PRINT "INPUT [";PL;"]:";
120 INPUT I$
130 IF I$="" THEN 160
140 T = VAL(I$):IF T>0 AND T<5 THEN 160
150 PRINT "[ERR] OUT OF RANGE 1 TO 4":GOTO 60
160 PL=T:POKE <???>,PL
170 ON PL=1 GOTO 210
180 ON PL=2 GOTO 220
190 ON PL=3 GOTO 230
200 ON PL=4 GOTO 240
210 LOAD "1520.BAS",8
220 LOAD "7470A.BAS",8
230 LOAD "DXY980.BAS",8
230 LOAD "DPX3300.BAS",8

170
20 REM INITILISE VARIABLES
25 DIM SINE(450)
30 NSH=50:REM NUMBER OF SHAPES
30 MINSID=3:REM MINIMUM NUMBER OF SIDES
40 MAXSID=12:REM MAXIMUM NUMBER OF SIDES
50 MINSIZ=10:REM MINIMUM SIZE OF SHAPE
60 MAXSIZ=100:REM MAXIMUM SIZE OF SHAPE
70 MINX=0:REM MINIMUM X LOCATION
80 MAXX=479:REM MAXIMUM X LOCATION
90 MINY=-998:REM MINIMUM Y LOCATION
100 MAXY=998:REM MAXIMUM Y LOCATION
110 FRAME="Y":REM DRAW A FRAME
140 PRINT "NUMBER OF SHAPES [";NSH;"]:";
150 INPUT I$
160 IF I$="" THEN 180
170 NSH = VAL(I$):IF NSH<0 THEN NSH=-NSH
180 PRINT "MINIMUM NUMBER OF SIDES [";MINSID;"]:";
190 INPUT I$
200 IF I$="" THEN 240
210 V=VAL(I$):IF V>2 THEN 230
220 PRINT "[ERR] NEEDS TO BE MORE THAN 2":GOTO 180
230 MINSID=V
240 PRINT "MAXIMUM NUMBER OF SIDES [";MAXSID;"]:";
250 INPUT I$
260 IF I$="" THEN 300
270 V=VAL(I$):IF V<MINSID THEN 290
280 PRINT "[ERR] NEEDS TO BE ";MINSID;"-";MAXSID:GOTO 240
290 MAXSID=V
300

999 REM DRAW SHAPES
1000 FOR I= 1 TO NS

1999 REM CALCULATE SHAPE
2000 FOR J= 0 TO NSID-1
2010 ANGLE=J*360/NSID+ROT
2010 X(J)=R*SINE(INT(ANGLE))+XLOC
2020 Y(J)=R*SINE(INT(ANGLE)+90)+YLOC
2030 NEXT J

4990 END
4999 REM INITIALISE SIN/COS LOOKUP TABLE
5000 FOR I=0 TO 450
5010 SINE(I)=SIN(I)
5020 NEXT I
5030 RETURN