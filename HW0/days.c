#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

struct date {
    int day;
    int month;
    int year;
};


 long int days(struct date input){

    //creating a date structure for the turn of the century 01/01/2000
    struct date start;
    start.day = 1;
    start.month = 1;
    start.year = 2000;

    //number of days from the turn of the century
    long int total_days = 0;

    //check if the date that was input is valid
    if((input.year >= 2000 && input.year <= 2099) && (input.month >= 1 && input.month <= 12) && (input.day >= 1 && input.day <= 30)){

        //add number of years in days to total
        total_days = (input.year - start.year)*360;
        //add number of days in months to total
        total_days += (input.month - start.month)*30;
        //add days to total
        total_days += (input.day- start.day);

        printf("%d\n", total_days);
        return total_days;

    }

    //if date is not valid return -1
    else{
        printf("Invalid date passed to argument, please enter a valid date\n");
        return -1;
    }


}

int main(int argc, char** argv) {

    //3 dates of invalid format
    struct date date1 = {0, 1, 2002};
    struct date date2 = {1, 0, 2002};
    struct date date3 = {1, 1, 1900};

    // 2 dates of valid format
    struct date date4 = {1, 1, 2002};
    struct date date5 = {30, 12, 2099};

    days(date1);
    days(date2);
    days(date3);
    days(date4);
    days(date5);


return 0;
}
