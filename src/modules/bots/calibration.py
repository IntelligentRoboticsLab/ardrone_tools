import cv,time,sys

def calibrate( bot, n_boards = 1, board_w = 6, board_h = 9 ):
	
	
	board_n  = board_w*board_h	# no of total corners
	board_sz = (board_w,board_h)	#size of board

	# creation of memory storages
	image_points		= cv.CreateMat(n_boards*board_n,2,cv.CV_32FC1)
	object_points		= cv.CreateMat(n_boards*board_n,3,cv.CV_32FC1)
	point_counts		= cv.CreateMat(n_boards,1,cv.CV_32SC1)
	intrinsic_matrix	= cv.CreateMat(3,3,cv.CV_32FC1)
	distortion_coefficient	= cv.CreateMat(5,1,cv.CV_32FC1)

	# capture frames of specified properties and modification of matrix values
	i = 0
	z = 0	# to print number of frames
	successes = 0

	cv.StartWindowThread()
	cv.NamedWindow( "Calibration" )
	cv.NamedWindow( "Last" )
	
	# capturing required number of views
	while(successes<n_boards):
		
		found		= 0
		image		= bot.get_cv_image()
		if not image:
			if cv.WaitKey(4) == 27:
				cv.DestroyAllWindows()
				return False
			else:
				continue
		gray_image	= cv.CreateImage(cv.GetSize(image),8,1)
		cv.CvtColor(image,gray_image,cv.CV_BGR2GRAY)

		(found,corners)	= cv.FindChessboardCorners(gray_image,board_sz,cv.CV_CALIB_CB_ADAPTIVE_THRESH| cv.CV_CALIB_CB_FILTER_QUADS)
		corners		= cv.FindCornerSubPix(gray_image,corners,(11,11),(-1,-1),(cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER,30,0.1))
		# if got a good image,draw chess board
		if found==1:
			print "found frame number {0}".format(z+1)
			cv.DrawChessboardCorners(image,board_sz,corners,1)
			corner_count = len(corners)
			z = z+1
		cv.ShowImage("Calibration",image)

		key = cv.WaitKey(33);
		if key == 27:
		    cv.DestroyAllWindows()
		    return False 
		elif ( key != 111 and key != 79 ) or found == 0:
		    continue 

		
		cv.ShowImage("Last",image)
		
		# if got a good image, add to matrix
		if len(corners)==board_n:
			step	= successes*board_n
			k	= step
			for j in range(board_n):
				cv.Set2D(image_points,k,0,corners[j][0])
				cv.Set2D(image_points,k,1,corners[j][1])
				cv.Set2D(object_points,k,0,float(j)/float(board_w))
				cv.Set2D(object_points,k,1,float(j)%float(board_w))
				cv.Set2D(object_points,k,2,0.0)
				k = k+1
			cv.Set2D(point_counts,successes,0,board_n)
			successes = successes+1
			print "-------------------------------------------------"
			print "\n"
		

	print "checking is fine ,all matrices are created"
	cv.DestroyWindow("Last")

	# now assigning new matrices according to view_count
	object_points2	= cv.CreateMat(successes*board_n,3,cv.CV_32FC1)
	image_points2	= cv.CreateMat(successes*board_n,2,cv.CV_32FC1)
	point_counts2	= cv.CreateMat(successes,1,cv.CV_32SC1)

	#transfer points to matrices

	for i in range(successes*board_n):
		cv.Set2D(image_points2,i,0,cv.Get2D(image_points,i,0))
		cv.Set2D(image_points2,i,1,cv.Get2D(image_points,i,1))
		cv.Set2D(object_points2,i,0,cv.Get2D(object_points,i,0))
		cv.Set2D(object_points2,i,1,cv.Get2D(object_points,i,1))
		cv.Set2D(object_points2,i,2,cv.Get2D(object_points,i,2))
	for i in range(successes):
		cv.Set2D(point_counts2,i,0,cv.Get2D(point_counts,i,0))

	cv.Set2D(intrinsic_matrix,0,0,1.0)
	cv.Set2D(intrinsic_matrix,1,1,1.0)

	rcv = cv.CreateMat(n_boards, 3, cv.CV_64FC1)
	tcv = cv.CreateMat(n_boards, 3, cv.CV_64FC1)

	print "checking camera calibration............."
	# camera calibration
	cv.CalibrateCamera2(object_points2,image_points2,point_counts2,cv.GetSize(image),intrinsic_matrix,distortion_coefficient,rcv,tcv,0)
	print " checking camera calibration.........................OK "	

	# storing results in xml files
	cv.Save( bot.NAME + "Intrinsics.xml", intrinsic_matrix)
	cv.Save( bot.NAME + "Distortion.xml", distortion_coefficient)
	# Loading from xml files
	intrinsic, distortion = get_calibration( bot.NAME )
	print " loaded all distortion parameters"

	mapx = cv.CreateImage( cv.GetSize(image), cv.IPL_DEPTH_32F, 1 );
	mapy = cv.CreateImage( cv.GetSize(image), cv.IPL_DEPTH_32F, 1 );
	cv.InitUndistortMap(intrinsic,distortion,mapx,mapy)
	cv.NamedWindow( "Undistort" )

	while cv.WaitKey(4) != 27 :
		image	= bot.get_cv_image()
		if not image:
			continue
		t	= cv.CreateImage(cv.GetSize(image), cv.IPL_DEPTH_8U, 3);
		cv.Copy( image, t)
		cv.ShowImage( "Calibration", image )
		cv.Remap( t, image, mapx, mapy )
		cv.ShowImage("Undistort", image)
		
		
	cv.DestroyAllWindows()

def get_calibration( name ):
    try:
	intrinsic  = cv.Load( "Intrinsics.xml" )
	distortion = cv.Load( "Distortion.xml" )
	return (intrinsic, distortion)
    except :
	return False, False


	
