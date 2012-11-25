' ***************************************************************************
' * The Virtual Force Field (VFF) method was developed by Johann Borenstein *
' * and Yoram Koren at the the university of Michigan in 1989.              *                        *
' * For further reference about the VFF method visit                        *
' * http://www-personal.engin.umich.edu/~johannb/vff&vfh.htm                *
' ***************************************************************************

Sub Main

	Dim PI,Fcr,Fct,X_target,Y_target,X,Y As Single
	Dim X_grid, Y_grid, i, j, C As Integer
	Dim Frx,Fry,d, dist_targ, rot, Fx, Fy As Single
	Dim Fcx,Fcy, Rx,Ry As Single

	PI=3.1415927
	Fcr= 0.2  	'Force constant (repelling)
	Fct= 0.4	'Force constant (attraction to the target)

	X_target=GetMarkX(0)  'Target coordinates (mark 0)
	Y_target=GetMarkY(0)

	SetCellSize(0,0.1) 'Set cell size 10 cm x 10 cm

	SetTimeStep(0.05)  'Set simulation time step of 0.1 seconds

	Do					' Start main loop

		X=GetMobotX(0)	' Present mobot coordinates (in meters)
		Y=GetMobotY(0)

		X_grid=CoordToGrid(0,X) ' indexes of cells where the
		Y_grid=CoordToGrid(0,Y)	' mobot center is

		MeasureRange(0,-1,3)	' Perform a range scan and update
								' the Certainty Grid (max. cell value=3)

		Frx=0	' Repulsive Force (x component)
		Fry=0	' Repulsive Force (y component)

		' Each ocuppied cell inside the windows of 33 x 33 cells
		' applies a repulsive force to the mobot.

		For i=X_grid-16 To X_grid+16
			For j=Y_grid-16 To Y_grid+16
				C=GetCell(0,i,j)
				If C<>0 Then
					d=Sqr((X_grid-i)^2+(Y_grid-j)^2)
					If d<>0 Then
						Frx=Frx+Fcr*C/d^2*(X_grid-i)/d
						Fry=Fry+Fcr*C/d^2*(Y_grid-j)/d
					End If
				End If
			Next
		Next

		dist_targ=Sqr((X-X_target)^2+(Y-Y_target)^2)

		' The target generates a constant-magnitud attracting force

		Fcx=Fct*(X_target-X)/dist_targ
		Fcy=Fct*(Y_target-Y)/dist_targ

		Rx=Frx+Fcx	' Resultant Force Vector
		Ry=Fry+Fcy

		rot=RotationalDiff(0,X+Rx,Y+Ry) 'shortest rotational difference between
										'current direction of travel and
										'direction of vector R

		SetSteering(0,0.5,3*rot)	'mobot turns into the direction of R
									'at constant speed and steering rate
									'proportional to the rotational difference

		StepForward		' Dynamics simulation progresses one time step

	Loop Until dist_targ<0.1	'Loop until mobot reaches the target


End Sub
