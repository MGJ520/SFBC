#include <stdio.h>

// ��������������������
#define MAX_ROWS 10
#define MAX_COLS 5

// ���������ݵĽṹ
typedef struct {
    char header[MAX_COLS][50]; // ��ͷ
    char content[MAX_ROWS][MAX_COLS][50]; // �������
    int row_count; // ��ǰ����
    int col_count; // ��ǰ����
} Table;

// ��ʼ�����
void initTable(Table* table, int rows, int cols) {
    table->row_count = rows;
    table->col_count = cols;
    for (int i = 0; i < cols; i++) {
        table->header[i][0] = '\0'; // ��ʼ����ͷΪ��
    }
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            table->content[i][j][0] = '\0'; // ��ʼ������Ϊ��
        }
    }
}

// ���ñ�ͷ
void setTableHeader(Table* table, int col, const char* header) {
    if (col >= 0 && col < table->col_count) {
        snprintf(table->header[col], sizeof(table->header[col]), "%s", header);
    }
}

// ���ñ������
void setTableCell(Table* table, int row, int col, const char* content) {
    if (row >= 0 && row < table->row_count && col >= 0 && col < table->col_count) {
        snprintf(table->content[row][col], sizeof(table->content[row][col]), "%s", content);
    }
}

// ������
void printTable(const Table* table) {
    // ��ӡ��ͷ
    for (int i = 0; i < table->col_count; i++) {
        printf("%-20s", table->header[i]);
    }
    printf("\n");
    printf("===============================\n");

    // ��ӡ����
    for (int i = 0; i < table->row_count; i++) {
        for (int j = 0; j < table->col_count; j++) {
            printf("%-20s", table->content[i][j]);
        }
        printf("\n");
    }
}

int main() {
    Table myTable;
    initTable(&myTable, 2, 3); // ��ʼ�����Ϊ2��2��

    // ���ñ�ͷ
    setTableHeader(&myTable, 0, "��Ŀ");
    setTableHeader(&myTable, 1, "����");

    // ���ñ������
    setTableCell(&myTable, 0, 0, "�����˹����������");
    setTableCell(&myTable, 0, 1, "2808���");
    setTableCell(&myTable, 1, 0, "������");
    setTableCell(&myTable, 1, 1, "7��");

    // ������
    printTable(&myTable);

    return 0;
}