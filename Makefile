all:
	cc -o sercat sercat.c

static:
	cc -o sercat --static sercat.c

