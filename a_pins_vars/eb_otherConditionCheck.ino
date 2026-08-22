

void conditions_manager()
{
  if (otherconditionsCheck())
  {
    myledwhiteon();
    otherconditionsDO();
    Taction = millis();
    otherconditionsCounter++;
    pathSteps++;
  }
  else
    ELSE();
}
boolean otherconditionsCheck()
{

  switch (otherconditionsCounter)
  {


      break;     

      case 0:
      return (compare(IntDsensors, "111111111xxxxx00"));

      break;
    case 1:
     
       return (compare(IntDsensors, "111111111xxxxx00"));
      break;
        case 2:
   
return (compare(IntDsensors, "xx11xxxxxxxxxx00"));
      break;
      case 3:

     
     return (compare(IntDsensors, "00xxxxxxxxxx11xx"));


      break;
    case 4:
      Serial.println(" Condition 3 check ");
      return (compare(IntDsensors, "xx11xxxxxxxxxx00"));
        //return compare(IntDsensors, "xxxx11111111xxxx");
      //return (CountLines() >= 2 || compare(IntDsensors, "1x111x000000"));
      break;
    case 5:
      Serial.println(" Condition 4 check ");
      return (compare(IntDsensors, "1111111xxxxxxx00"));
      break;

    case 6:
      Serial.println(" Condition 3 check ");
      return (compare(IntDsensors, "xx11111xxxxxxx00")||CountLines() >= 2);

      // || compare(IntDsensors, "000000000000")) ;
      //return CountLines() >= 2;
      break;

    case 7:
      Serial.println(" Condition 5 check ");
          return CountLines() >= 2;
        
      break;
    case 8:
      Serial.println(" Condition 5 check ");
        return compare(IntDsensors, "xx11111xxxxxxx00");
      break;
//    case 6:
//      Serial.println(" Condition 5 check ");
//      return CountLines() >= 2;
//      break;
//    case 7:
//      Serial.println(" Condition 5 check ");
//      return true ;
//      break;
    case 9:
      Serial.println(" Condition 5 check ");
       return (CountLines() >= 2 );
      break;
    case 10:
      Serial.println(" Condition 5 check ");
    return compare(IntDsensors, "xxxx11111111xxxx");
      break;
    case 11:
      Serial.println(" Condition 5 check ");
      return compare(IntDsensors, "xxxx11111111xxxx");
      break;
    case 12:
      Serial.println(" Condition 5 check ");
      return compare(IntDsensors, "0000000000000000");
      break;
    case 13:
      Serial.println(" Condition 5 check ");
      return ( compare(IntDsensors, "xxxx1111111xxxx") );
      break;
       case 14:
      Serial.println(" Condition 5 check ");
      
      return CountLines() >= 2;
      break;
       case 15:
      Serial.println(" Condition 5 check ");
      return CountLines() >= 2;
      break;
       case 16:
      Serial.println(" Condition 5 check ");
       return CountLines() >= 2;
      break;
       case 17:
      Serial.println(" Condition 5 check ");
      return CountLines() >= 2;
      break;
      case 18:
      Serial.println(" Condition 5 check ");
      return true;
      break;
      case 19:
      Serial.println(" Condition 5 check ");
      return true;
      break;
      case 20:
      Serial.println(" Condition 5 check ");
      return true;
      break;
      case 21:
      Serial.println(" Condition 5 check ");
      return true;
      break;
      
    // ADD CASES IF U HAVE MORE
    default:
      Serial.println("Error! NOMBER OF C IN PATHSTRING IS MORE THAN THE CONDITIONS IN otherconditionsCheck FUNCTION");
      return true; // to continue following and does get stuck
  }
  return false; // if anything missing in the conditions bech lcode may7bsch
}
