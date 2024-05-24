#include <iostream>
#include <fstream>
#include <string>
#include <vector>

void removeLines(const std::string& filename, int startLine, int numLinesToRemove) {
    // 打开文件
    std::ifstream inputFile(filename);
    if (!inputFile.is_open()) {
        std::cerr << "无法打开文件：" << filename << std::endl;
        return;
    }

    // 读取文件内容到 vector
    std::vector<std::string> lines;
    std::string line;
    int lineNumber = 0;
    while (std::getline(inputFile, line)) {
        ++lineNumber;
        if (lineNumber < startLine || lineNumber >= startLine + numLinesToRemove) {
            lines.push_back(line);
        }
    }
    inputFile.close();

    // 将剩余的内容写回文件
    std::ofstream outputFile(filename);
    if (!outputFile.is_open()) {
        std::cerr << "无法打开文件进行写入：" << filename << std::endl;
        return;
    }
    for (const std::string& l : lines) {
        outputFile << l << std::endl;
    }
    outputFile.close();
}

int main() {
    std::string filename = "../example.txt";
    int startLine = 10; // 起始行号
    int numLinesToRemove = 5; // 要删除的行数

    removeLines(filename, startLine, numLinesToRemove);

    std::cout << "已删除文件 " << filename << " 中从第 " << startLine << " 行开始的 " << numLinesToRemove << " 行代码。" << std::endl;

    return 0;
}

