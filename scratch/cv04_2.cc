#include <ns3/log.h>
#include <sqlite3.h>
#include <cstdio>
#include <cstring>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("cvicenie 4.2");

static int callback(void *NotUsed, int argc, char **argv, char **azColName) {
   int i;
   for(i = 0; i<argc; i++) {
      printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
   }
   printf("\n");
   return 0;
}

int main (int argc, char *argv[]) {
  sqlite3 *db;
   char *zErrMsg = 0;
   int rc;

   rc = sqlite3_open("testDB.db", &db);

   if (rc) {
      fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
      return(0);
   }


   fprintf(stderr, "Opened database successfully\n");

   /* Create SQL statement */
   const char* sql = "CREATE TABLE COMPANY("  \
                     "ID INT PRIMARY KEY     NOT NULL," \
                     "NAME           TEXT    NOT NULL," \
                     "AGE            INT     NOT NULL," \
                     "ADDRESS        CHAR(50)," \
                     "SALARY         REAL );";

   /* Execute SQL statement */
   rc = sqlite3_exec(db, sql, callback, 0, &zErrMsg);

   if( rc != SQLITE_OK ){
      fprintf(stderr, "SQL error: %s\n", zErrMsg);
      sqlite3_free(zErrMsg);
   }
   else
      fprintf(stdout, "Table created successfully\n");

   sqlite3_close(db);

   return 0;
}
