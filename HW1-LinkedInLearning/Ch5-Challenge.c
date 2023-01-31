#include <stdio.h>
#include <stdlib.h>

//Create newline function
void newline() { //This function will create a newline. Seems unneccesary CJS.
    putchar ('\n');
}

void separator_row(int repeatCount, char repeatChar) { //This function will create a line of however many chars are inputed at that frequency CJS.
    for( int x=0; x<repeatCount; x++ ) //Loop repeatCount times CJS.
			putchar(repeatChar); //Put the repeatChar CJS.
			newline(); //Call the above newline function CJS.
}

/* generate multiple lines of output */
int main()
{
	int row,c,rows,x;
	char column;

	/* get and verify input */
	printf("How many rows (18 max)? ");
	scanf("%d",&rows);
	/* avoid out-of-range values */
	if( rows>18 )
		rows = 18;

	/* use variable c to count the number rows */
	c = 0;
	/* process the rows */
	for( row=0; row<rows; row++ )
	{
		/* switch output every other row */
		/* separator row */
		if( row%2 )
		{
			separator_row(40, '='); //Call separator row function CJS.
		}
		else
		/* values row */
		{
			for( column='A'; column<='J'; column++ )
				printf(" %d%c ",c,column);
			newline(); //Call newline function CJS.
			c++;
		}
	}

	return 0;
}
