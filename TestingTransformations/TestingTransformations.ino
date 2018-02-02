class Matrix {
  public:
    static constexpr int rows = 3;
    static constexpr int cols = rows;
  
    Matrix() {}
  
    void setVals(double input[]) {
      for (uint8_t i = 0; i < rows * cols; i++) {
        _values[i/rows][i % cols] = input[i];
      }
    }

    double at(int row, int col);

    static Matrix multiply(Matrix& a, Matrix& b) {
      Matrix result;
      double vals[a.rows * b.cols];
      for (uint8_t i = 0; i < a.rows; i++) {
        for (uint8_t j = 0; j < b.cols; j++) {
          double dotProduct = 0;
          for (uint8_t k = 0; k < b.cols; k++) {
            dotProduct += a.at(i, j) * b.at(j, i);
          }
        }
      }

      result.setVals(vals);
      return result;
    }

    void printValues() {
      Serial.println();
      for (uint8_t row = 0; row < rows; row++) {
        Serial.print("[");
        for (uint8_t col = 0; col < cols; col++) {
          Serial.print(_values[row][col]);
          Serial.print(" ");
        }
        Serial.println("]");
      }
    }

  private:
    double _values[rows][cols];
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Matrix matrix;
  double values[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  for (uint8_t i = 0; i < sizeof(values)/sizeof(values[0]); i++) {
    Serial.print(values[i]);
    Serial.print(" ");
  }

  Serial.println();
  matrix.setVals(values);

  matrix.printValues();
}

void loop() {
  // put your main code here, to run repeatedly:
}
