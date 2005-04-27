/*********************************************************************
** Title:		stddef.h - (ANSI) Standard Definitions for wizC.
**
** Author:		Marcel van Lieshout
**
** Copyright:	(c) 2005, HMCS, Marcel van Lieshout
**
** License:		This software is released to the public domain and comes
**				without	warranty and/or guarantees of any kind. You have
**				the right to use, copy, modify and/or (re-)distribute the
**				software as long as the reference to the author is
**				maintained in the software and a reference to the author
**				is included in any documentation of each product in which
**				this library (in it's original or in a modified form)
**				is used.
*********************************************************************/
#ifndef _STDDEF_H
#define _STDDEF_H

typedef	int				ptrdiff_t;
typedef	unsigned int	size_t;
typedef	unsigned char	wchar_t;

#ifndef NULL
	#define	NULL	(0)
#endif

#define	offsetof(s,m)	((size_t)&(((s *)0)->m))

#endif	/* _STDDEF_H */
