//This function evaluates if two floats are close enough, using an epsilon criteria

bool aroundEqual(float a,float b, float epsilon){
  if(epsilon < 0){
    epsilon = -epsilon;
  }
  if(a <= b + epsilon){
    if (a >= b - epsilon){
      return true;
    }
  }
  return false;
}
