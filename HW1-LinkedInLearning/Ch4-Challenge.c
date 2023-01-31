#include <stdio.h>
#include <stdlib.h>

/* generate multiple lines of output */
int main()
{
	int a,rows;
    int row; //Create row and column global variables. CJS
    char column;
    int rowCounter, x;

	/* get and verify input */
	printf("How many rows (18 max)? ");
	scanf("%d",&rows);
	/* avoid out-of-range values */
	if( rows>18 )
		rows = 18;

	// process the rows

    //Outside loop process rows CJS
    rowCounter=0;
    for (row = 0; row<rows; row++) {
        if (row % 2){ //If even row, write ==========
            for (x=0; x<40; x++)
                putchar('='); //Put 40 = signs
            putchar('\n'); //make a new line.
        }
        else {
            for (column='A'; column<='J'; column++)  //For columns in range of A to J do...
                printf(" %d%c", rowCounter, column); //Print each element
            putchar('\n'); //Make a newline.
            rowCounter++; //Inciment the rowCounter.
        }
    }  
	return 0;
}
