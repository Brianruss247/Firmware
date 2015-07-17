#
# Estimator block for UAVbook http://uavbook.byu.edu/doku.php
#

MODULE_COMMAND		= estimator
SRCS			= estimator_main.cpp \
				estimator_base.cpp \
				estimator_example.cpp \
				estimator_params.c

MODULE_STACKSIZE 	= 4000

EXTRACXXFLAGS	= -Wframe-larger-than=2400
