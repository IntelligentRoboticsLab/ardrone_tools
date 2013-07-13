import ardrone
import cv
from bot import *


def is_position_aligned( line, image_size, marge ):
    wanted = ( (line[0][0] + line[1][0])/2 , (line[0][1] + line[1][1])/2 )
    at	   = ( image_size[0]/2 , image_size[1]/2 )
    return abs( wanted[0] - at[0], wanted[1] - at[1] ) < marge
'''
1 = top
2 = right
3 = bottom
4 = left
'''
def align_position( line, direction, image_size ):
    if direction == 1 :
	if line[0][1] < line[1][1] :
	    wanted = line[0]
	else :
	    wanted = line[1]
    elif direction == 2 :
	if line[0][0] > line[1][0] :
	    wanted = line[0]
	else :
	    wanted = line[1]
    elif direction == 3 :
	if line[0][1] > line[1][1] :
	    wanted = line[0]
	else :
	    wanted = line[1]
    elif direction == 4 :
	if line[0][0] < line[1][0] :
	    wanted = line[0]
	else :
	    wanted = line[1]
    at = ( image_size[0]/2 , image_size[1]/2 )
    return ( wanted[0] - at[0], wanted[1] - at[1] )

def wanted_angle( direction ):
    if direction == 0 or direction == 2 :
    	return 0
    return Math.pi/2
'''
1 = top
2 = right
3 = bottom
4 = left
'''
def align_rotation( line, direction, rot_marge ):
    alpha = Math.atan2( (line[0][0] - line[1][0])/(line[0][1] - line[1][1]) )
    if alpha > Math.pi/2 :
	alpha = alpha - Math.pi/2
    beta = wanted_angle( direction ) - alpha
    if abs( beta ) > Math.pi/2 - abs(beta) :
	beta = Math.pi/2 - abs( beta )
    if abs(beta) < rot_marge :
	return 0
    return beta
      
def aligner( bot, direction, speed = 1, angle_speed = 1, marge = 70, rot_marge = 0.2 ):
    image_size = cv.GetSize(bot.get_cv_image())
    if is_position_aligned( line, image_size, marge ):
	max_distance = (image_size[0]**2 + image_size[1]**2)**0.5
	vector = align_position( line, direction, image_size )
	bot.set_control( linearx = vector[0] * speed / max_distance )
	bot.set_control( lineary = vector[1] * speed / max_distance )
	return False
    bot.set_control( linearx = 0 )
    bot.set_control( lineary = 0 )
    alpha = align_rotation( line, direction, rot_marge )
    if not alpha :
	return True
    else :
	bot.set_control( angularz = alpha * speed / Math.pi/2 )

