#define _CRT_SECURE_NO_WARNINGS
#define MAX_LEN 100000
#include <stdio.h>
#include <string.h>
#include <mysql/mysql.h>
#include <errno.h>
#include <stdlib.h>
#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>
#define path "/home/miman/Desktop/MIMAN_GS/data/beacon"
// upgraded from read_and_add5.c : change data type

void main(void) {

    /*----------------Scan beacon folder----------------------
                    -> store whole 'beacon file name' in array*/
    struct dirent **namelist;
    int count;
    int idx;

    if ((count=scandir(path,&namelist,NULL, alphasort))==-1) {
    //red line under alphasort is ok (not cause any error)
        fprintf(stderr, "%s Scan Error: %s\n",path,strerror(errno));
        //return 1;
    }

    /*---------------------Connect mysql--------------------*/

    MYSQL mysql;
    MYSQL_RES *res;
    MYSQL_ROW row;
    MYSQL_FIELD *field_name;

    mysql_init(&mysql);

    if (!mysql_real_connect(&mysql,NULL,"miman","qwe123",NULL,3306,(char*)NULL,0)) {
        printf("%s\n",mysql_error(&mysql));
        exit(1);
    }

    if (mysql_query(&mysql,"use MIMAN")) {
        printf("%s\n",mysql_error(&mysql));
        exit(1);
    }
    
    /*--------------------Read each beacon file----------------------
                        -> iterate for # of beacon file */

    //char str[MAX_LEN];
    for (idx=2; idx<count-3; idx++) { //first two file is not right '.' '..'
        FILE* fp;
        char data[34][64];
        char str[MAX_LEN];
        //rivise directory.
        char dir[300];
        //sprintf(dir,"..\\Dsektop\\MIMAN_GS\\data\\beacon\\%s",namelist[idx]->d_name);
        //printf("%s\n",dir);
        fp = fopen(namelist[idx]->d_name, "r");

        
        int cnt = 0;
        
        while (fgets(str, MAX_LEN, fp) != NULL) {
            //printf("%s\n", str);
            strcpy(data[cnt], str);
            cnt++;
        }

        fclose(fp);
        
        /*------------------------Store data-----------------------------*/

        char *names[34]; //store field(=column) name  ex) Call sign 
        char *values[34]; //store data value
        int i;
        for (i = 0; i < 34; i++) {
            names[i] = strtok(data[i], ":");
            values[i] = strtok(NULL, ":");
        }
       
        /*-----------Making query sentence - for whole data except Binary.-----------*/         
        /* Mysql query Grammer : 
            insert into [table name](column1, column2...) values(value1, value2...) */
        
        // append integer data : 4,6,7,8,9,10,12,13,14,16,17,18,23,24,27,28,31,32,33,34
        char query1[1000];
        sprintf(query1,"insert into beacon(`Length`,`Spacecraft Time`,`Reboot Count`,`Current Mode`,`Current Submode`,`Previous Mode`,`Mode`,`Reset Cause`,`Reset Count`,`Battery IN Voltage`,`Baterry In Current`,`Battery Temperature`,`Mboard Supply V`,`Mboard Temperature`,`Dboard Channel I 5V`,`Dboard Channel I 12V`,`RW Error Count`,`Total RX Bytes`,`Boot Count`,`Active Conf`) values(%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d)",
        atoi(values[3]),atoi(values[5]),atoi(values[6]),atoi(values[7]),atoi(values[8]),atoi(values[9]),
        atoi(values[11]),atoi(values[12]),atoi(values[13]),atoi(values[15]),atoi(values[16]),atoi(values[17]),
        atoi(values[22]),atoi(values[23]),atoi(values[26]),atoi(values[27]),atoi(values[30]),atoi(values[31]),
        atoi(values[32]),atoi(values[33]));
        // Get row number - used in 'update' query :query2,3,4,5
        if (mysql_query(&mysql,"select count(*) from beacon")) {
            printf("%s\n",mysql_error(&mysql));
            exit(1);
        }
        res=mysql_use_result(&mysql);
        row=mysql_fetch_row(res);
        printf("%d\n",atoi(row[0])); // row[0] is char. use atoi to convert int & used for loading check

        // append char type data : 1,11
        char query2[100];
        sprintf(query2,
        "update beacon set `Call Sign`='MIMAN',`Previous SubMode`= '%s' where No=%d",
        values[10],atoi(row[0]));

        // append hex type data : 2,3,15
        char query3[200];
        sprintf(query3,
        "update beacon set `Message ID` = %ld,`Sequence` = %ld,`Battery Status` = %ld where No=%d",
        strtol(values[1],NULL,16),strtol(values[2],NULL,16),strtol(values[14],NULL,16),atoi(row[0]));

        // append float & multi value data : 29 & 5, 19, 20, 21, 22, 25, 26, 30
        char query4[500];
        sprintf(query4,
        "update beacon set `Temperature`= %f, `Time Code`=\"%s\", `Mboard MPPT In V`=\"%s\", `Mboard MPPT In I`=\"%s\", `Mboard Channel I 3V`=\"%s\", `Mboard Channel I 5V`=\"%s\", `Dboard MPPT In V`=\"%s\", `Dboard MPPT In I`=\"%s\", `Angular Velocity`=\"%s\" where No=%d",
        strtof(values[28],NULL),values[4],values[18],values[19],values[20],values[21],values[24],values[25],values[29],atoi(row[0]));

        
        mysql_free_result(res);
        
        
    /*---------------------------Main query - insert data----------------------------*/

        // insert beacon data into table 'beacon_data'
        if (mysql_query(&mysql,query1)) {   
            printf("%s\n",mysql_error(&mysql));
            exit(1);
        }
        if (mysql_query(&mysql,query2)) {
            printf("%s\n", mysql_error(&mysql));
            exit(1);
        }
        if (mysql_query(&mysql,query3)) {
            printf("%s\n",mysql_error(&mysql));
            exit(1);
        }
        if (mysql_query(&mysql,query4)) {
            printf("%s\n",mysql_error(&mysql));
            exit(1);
        }
        //chek process
        printf("%d\n",idx);
    }

    /*---------------------Print out whole database---------------------- */
    
    
    if (mysql_query(&mysql,"select * from beacon")) { // show all data
        printf("%s\n",mysql_error(&mysql));
        exit(1);
    }
    res=mysql_store_result(&mysql);
    /*mysql_store_result() reads the entire result of a query to the client, 
    allocates a MYSQL_RES structure, 
    and places the result into this structure. */
    int fields;

    fields=mysql_num_fields(res);
    // printf("number of fields: %d\n",fields);
    int cnt2;
    while ((row=mysql_fetch_row(res))) {
        for (cnt2=0;cnt2<fields;cnt2++) {
            if (cnt2==0) {
                while (field_name=mysql_fetch_field(res)) {
                    printf("%20s",field_name->name);
                }
                printf("\n");
            }
            printf("%20s",row[cnt2]);
        }

        printf("\n");
    } 
    mysql_free_result(res);
    

    // release namelist memory
    for (idx=0; idx<count; idx++) {
        free(namelist[idx]);
    }
    free(namelist);
    mysql_close(&mysql);
}
