/* MATRIX3 */

Matrix3::Matrix3 () {
    X = Vector3(1.0, 0.0, 0.0);
    Y = Vector3(0.0, 1.0, 0.0);
    Z = Vector3(0.0, 0.0, 1.0);
}

void Matrix3::rotate(Vector3 *v) {
  Vector3 temp = *v;
   v->x = X.x * temp.x + X.y * temp.y + X.z * temp.z;
   v->y = Y.x * temp.x + Y.y * temp.y + Y.z * temp.z;
   v->z = Z.x * temp.x + Z.y * temp.y + Z.z * temp.z;
}

void Matrix3::unRotate(Vector3 *v) {
   Vector3 temp = *v; 
   v->x = X.x * temp.x + Y.x * temp.y + Z.x * temp.z;
   v->y = X.y * temp.x + Y.y * temp.y + Z.y * temp.z;
   v->z = X.z * temp.x + Y.z * temp.y + Z.z * temp.z;
}

void Matrix3::print() {
  X.print();
  Serial.print(",");
  Y.print();
  Serial.print(",");
  Z.print();
}


/* VECTORS */

Vector3::Vector3 () {
  x = 0.0;
  y = 0.0;
  z = 0.0;
}

Vector3::Vector3 (float X, float Y, float Z) {
  x = X;
  y = Y;
  z = Z;
}

const Vector3 Vector3::operator+(const Vector3 &other) const {
  return Vector3(x + other.x, y + other.y, z + other.z);
}

const Vector3 Vector3::operator-(const Vector3 &other) const {
  return Vector3(x - other.x, y - other.y, z - other.z);
}
  
const float Vector3::dot(const Vector3 &other) const {
  return  (x * other.x + y * other.y + z * other.z);
}

const Vector3 Vector3::cross(const Vector3 &other) const {
  return Vector3(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
}

Vector3 & Vector3::operator+=(const Vector3 &other) {
  x += other.x;
  y += other.y;
  z += other.z;
  return *this;
}

Vector3 & Vector3::operator-=(const Vector3 &other) {
  x -= other.x;
  y -= other.y;
  z -= other.z;
  return *this;
}


const Vector3 Vector3::operator/(const Vector3 &other) const {
  return Vector3 (x / other.x, y / other.y, z / other.z);
}

const Vector3 Vector3::operator/(const float scale) const{
  return Vector3 (x / scale, y / scale, z / scale);
}

Vector3 & Vector3::operator/=(const Vector3  &other) {
  x /= other.x;
  y /= other.y;
  z /= other.z;
  return *this;
}

Vector3 & Vector3::operator/=(const float scale) {
  x /= scale;
  y /= scale;
  z /= scale;
  return *this;
}
const Vector3 Vector3::operator*(const float scale) const {
  return Vector3 (scale * x, scale * y, scale * z);
}


Vector3 & Vector3::operator*=(const float scale) {
  x *= scale;
  y *= scale;
  z *= scale;
  return *this;
}

float Vector3::magnitude() {
  return sqrt(x*x + y*y + z*z);
}

const Vector3 Vector3::squared() const {
  return Vector3 (x * x, y * y, z * z);
}

const Vector3 Vector3::normalize() {
  float myMag = sqrt(x*x + y*y + z*z);
  return Vector3 (x / myMag, y / myMag, z / myMag);
  
}
  
void Vector3::print() {
  Serial.print(1e5*x,0);
  Serial.print(",");
  Serial.print(1e5*y,0);
  Serial.print(",");
  Serial.print(1e5*z,0);
}


/* VECTOR2 */

Vector2::Vector2() {
  u = 0.0;
  v = 0.0;
}

Vector2::Vector2(float A, float B) {
  u = A;
  v = B;
}

const Vector2 Vector2::operator+(const Vector2 &other) const {
  return Vector2(u + other.u, v + other.v);
}

const Vector2 Vector2::operator-(const Vector2 &other) const {
  return Vector2(u - other.u, v - other.v);
}

void Vector2::print() {
  Serial.print(u);
  Serial.print(",");
  Serial.print(v);
}


/* MATRIX2 */

Matrix2::Matrix2() {  
  uu = 0.0;
  uv = 0.0;
  vu = 0.0;
  vv = 0.0;
}

Matrix2::Matrix2(float A, float B, float C, float D) {
  uu = A;
  uv = B;
  vu = C;
  vv = D;
}

const Matrix2 Matrix2::operator+(const Matrix2 &other) const {
  return Matrix2( uu + other.uu, uv + other.uv, vu + other.vu, vv + other.vv);
}

const Matrix2 Matrix2::operator-(const Matrix2 &other) const {
  return Matrix2( uu - other.uu, uv - other.uv, vu - other.vu, vv - other.vv);
}

const Matrix2 Matrix2::operator*(const Matrix2 &other) const {
  return Matrix2( uu * other.uu + uv * other.vu,
                  uu * other.uv + uv * other.vv,
                  vu * other.uu + vv * other.vu,
                  vu * other.uv + vv * other.vv);
}

const Vector2 Matrix2::operator*(const Vector2 &vec) const {
  return Vector2( uu * vec.u + uv * vec.v, vu * vec.u + vv * vec.v);
}

const Matrix2 Matrix2::operator*(const float scale) const {
  return Matrix2( uu * scale, uv * scale, vu * scale, vv * scale);
}

const Matrix2 Matrix2::T() const {
  return Matrix2 (uu, vu, uv, vv);
}

const Matrix2 Matrix2::I() const {
  float det = uu * vv - uv * vu;
  return Matrix2( vv / det, -uv / det, -vu / det, uu / det);
}

void Matrix2::print() {
  Serial.print("[ ");
  Serial.print(uu,5);
  Serial.print(", ");
  Serial.print(uv,5);
  Serial.print(";  ");
  Serial.print(vu,5);
  Serial.print(", ");
  Serial.print(vv,5);
  Serial.print(" ]");
}

