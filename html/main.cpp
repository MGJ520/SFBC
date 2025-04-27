#include <stdio.h>

// 定义表格的最大行数和列数
#define MAX_ROWS 10
#define MAX_COLS 5

// 定义表格内容的结构
typedef struct {
    char header[MAX_COLS][50]; // 表头
    char content[MAX_ROWS][MAX_COLS][50]; // 表格内容
    int row_count; // 当前行数
    int col_count; // 当前列数
} Table;

// 初始化表格
void initTable(Table* table, int rows, int cols) {
    table->row_count = rows;
    table->col_count = cols;
    for (int i = 0; i < cols; i++) {
        table->header[i][0] = '\0'; // 初始化表头为空
    }
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            table->content[i][j][0] = '\0'; // 初始化内容为空
        }
    }
}

// 设置表头
void setTableHeader(Table* table, int col, const char* header) {
    if (col >= 0 && col < table->col_count) {
        snprintf(table->header[col], sizeof(table->header[col]), "%s", header);
    }
}

// 设置表格内容
void setTableCell(Table* table, int row, int col, const char* content) {
    if (row >= 0 && row < table->row_count && col >= 0 && col < table->col_count) {
        snprintf(table->content[row][col], sizeof(table->content[row][col]), "%s", content);
    }
}

// 输出表格
void printTable(const Table* table) {
    // 打印表头
    for (int i = 0; i < table->col_count; i++) {
        printf("%-20s", table->header[i]);
    }
    printf("\n");
    printf("===============================\n");

    // 打印内容
    for (int i = 0; i < table->row_count; i++) {
        for (int j = 0; j < table->col_count; j++) {
            printf("%-20s", table->content[i][j]);
        }
        printf("\n");
    }
}

int main() {
    Table myTable;
    initTable(&myTable, 2, 3); // 初始化表格为2行2列

    // 设置表头
    setTableHeader(&myTable, 0, "项目");
    setTableHeader(&myTable, 1, "内容");

    // 设置表格内容
    setTableCell(&myTable, 0, 0, "电机阿斯弗撒发生撒");
    setTableCell(&myTable, 0, 1, "2808电机");
    setTableCell(&myTable, 1, 0, "极对数");
    setTableCell(&myTable, 1, 1, "7对");

    // 输出表格
    printTable(&myTable);

    return 0;
}