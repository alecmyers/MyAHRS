

typedef struct{
  int x, y, z;
} intVector;


class Vector3 {
  public:
    float x, y, z;
    Vector3 ();
    Vector3 (float X, float Y, float Z);
    const Vector3 cross(const Vector3 &other) const;
    const Vector3 operator+(const Vector3 &other) const;
    const Vector3 operator-(const Vector3 &other) const;
    const Vector3 operator*(const float scale) const;
    const Vector3 operator/(const float scale) const;
    const Vector3 operator/(const Vector3 &other) const;
    
    Vector3 & operator*=(const float scale);
    Vector3 & operator/=(const float scale);
    Vector3 & operator/=(const Vector3 &other);
    Vector3 & operator+=(const Vector3 &other);
    Vector3 & operator-=(const Vector3 &other);
    float magnitude();
    const Vector3 normalize();
    const Vector3 squared() const;
    const float dot(const Vector3 &other) const;
    void print();
};


class Vector2 {
  public:
    float u,v;
    Vector2 ();
    Vector2 (float A, float B);
    const Vector2 operator+(const Vector2 &other) const;
    const Vector2 operator-(const Vector2 &other) const;
    void print();
};



class Matrix3 {
  public:
    Vector3 X, Y, Z;
    Matrix3();
    void rotate(Vector3 *v);
    void unRotate(Vector3 *v);
    void print();
};

class Matrix2 {
  public:
    float uu, uv, vu, vv;
    Matrix2();

    Matrix2(float A, float B, float C, float D);
    const Matrix2 operator+(const Matrix2 &other) const;
    const Matrix2 operator-(const Matrix2 &other) const;
    const Matrix2 operator*(const float scale) const;
    const Matrix2 operator*(const Matrix2 &other) const;
    const Vector2 operator*(const Vector2 &vec) const;
    const Matrix2 T() const;
    const Matrix2 I() const;
    void print();
};
