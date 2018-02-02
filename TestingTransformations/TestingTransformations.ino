#define WIDTH 6

class Vector {
  public:
    static constexpr uint8_t entries = WIDTH;

    Vector() {}

    void setVals(double input[]) {
      for (uint8_t i = 0; i < entries; i++) {
        _values[i] = input[i];
      }
    }

    double at(uint8_t entry) {
      return _values[entry];
    }

    double* getVals() {
      return _values;
    }

    void printValues() {}

  protected:
    double _values[entries];
};

class ColumnVector : public Vector {
  public:
    static constexpr uint8_t rows = entries;
    static constexpr uint8_t cols = 1;

    void printValues() {
      Serial.println();
      String rowStr;
      for (uint8_t row = 0; row < rows; row++) {
        rowStr = String("[" + String(at(row)) + "]");
        Serial.println(rowStr);
      }
      Serial.println();
    }
};

class RowVector : public Vector {
  public:
    static constexpr uint8_t rows = 1;
    static constexpr uint8_t cols = entries;

    void printValues() {
      Serial.println();
      String colStr;
      for (uint8_t col = 0; col < cols; col++) {
        colStr = String("[" + String(at(col)) + "]");
        Serial.println(colStr);
      }
      Serial.println();
    }
};

class DataMatrix {
  public:
    static constexpr uint8_t rows = WIDTH;
    static constexpr uint8_t cols = WIDTH;
  
    DataMatrix() {}
  
    void setVals(double input[]) {
      for (uint8_t i = 0; i < rows * cols; i++) {
        _values[i/rows][i % cols] = input[i];
      }
    }

    double at(uint8_t row, uint8_t col) {
      return _values[row][col];
    }

    double* getRow(uint8_t row) {
      static double result[cols];

      for (uint8_t i = 0; i < cols; i++) {
        result[i] = _values[row][i];
      }

      return result;
    }

    double* getCol(uint8_t col) {
      static double result[rows];

      for (uint8_t i = 0; i < rows; i++) {
        result[i] = _values[i][col];
      }

      return result;
    }

    void multiplyByScalar(double scalar) {
      for (uint8_t row = 0; row < rows; row++) {
        for (uint8_t col = 0; col < cols; col++) {
          _values[row][col] *= scalar;
        }
      }
    }

    void printValues() {
      Serial.println();
      String rowStr;
      for (uint8_t row = 0; row < rows; row++) {
        rowStr = String("[");
        for (uint8_t col = 0; col < cols; col++) {
          rowStr += String(at(row, col)) + " ";
        }
        rowStr.remove(rowStr.length() - 1);
        rowStr += String("]");

        Serial.println(rowStr);
      }
      Serial.println();
    }

  private:
    double _values[rows][cols];
};

namespace Util {
  static double getDotProduct(double aRow[], double bCol[], int numElements) {
    double result = 0;

    for (uint8_t i = 0; i < numElements; i++) {
      result += aRow[i] * bCol[i];
    }

    return result;
  }
  
  static DataMatrix multiply(DataMatrix& a, DataMatrix& b) {
    DataMatrix result;
    double vals[a.rows * b.cols];
  
    for (uint8_t row = 0; row < a.rows; row++) {
      for (uint8_t col = 0; col < b.cols; col++) {
        double* aRow = a.getRow(row);
        double* bCol = b.getCol(col);
        
        double dotProduct = getDotProduct(aRow, bCol, b.rows);
        
        vals[row*a.rows + col] = dotProduct;
      }
    }
  
    result.setVals(vals);
    return result;
  }

  static ColumnVector colMultiply(DataMatrix& a, ColumnVector& b) {
    ColumnVector result;
    double vals[b.rows];

    for (uint8_t row = 0; row < a.rows; row++) {
      double* matrixRow = a.getRow(row);
      vals[row] = getDotProduct(matrixRow, b.getVals(), b.rows);
    }

    result.setVals(vals);
    return result;
  }

  static RowVector rowMultiply(RowVector& a, DataMatrix& b) {
    RowVector result;
    double vals[a.cols];

    for (uint8_t col = 0; col < b.cols; col++) {
      double* matrixCol = b.getCol(col);
      vals[col] = getDotProduct(a.getVals(), matrixCol, a.cols);
    }

    result.setVals(vals);
    return result;
  }
  
  static DataMatrix add(DataMatrix& a, DataMatrix& b) {
    DataMatrix result;
    double vals[a.rows * a.cols];
  
    for (uint8_t i = 0; i < a.rows; i++) {
      for (uint8_t j = 0; j < a.cols; j++) {
        vals[i*a.rows + j] = a.at(i, j) + b.at(i,j);
      }
    }
  
    result.setVals(vals);
    return result;
  } 
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  DataMatrix A;
  double valuesA[WIDTH*WIDTH];
  for (uint8_t i = 0; i < WIDTH; i++) {
    for (uint8_t j = 0; j < WIDTH; j++) {
      uint8_t index = WIDTH*i + j;
      if (i == j) {
        valuesA[index] = 1;
      } else {
        valuesA[index] = 0;
      }
    }
  }

  A.setVals(valuesA);

  A.printValues();

  double valuesB[] = {15, 6, 43, 23, 71, 18};
  ColumnVector B;
  B.setVals(valuesB);

  B.printValues();

  //DataMatrix result = Util::add(A, B);
  //result.printValues();

  ColumnVector result2 = Util::colMultiply(A, B);
  result2.printValues();

  RowVector C;
  C.setVals(valuesB);
  C.printValues();

  RowVector result3 = Util::rowMultiply(C, A);
  result3.printValues();
}

void loop() {
  // put your main code here, to run repeatedly:
}
