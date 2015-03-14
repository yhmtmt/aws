#include <iostream>
#include "aws_cmd.h"

#define AWS_CMD "awstime"
#define AWS_CMD_USAGE "awstime <option>"

int main(int argc, char ** argv)
{
  return aws_cmd(argc, argv, AWS_CMD);
}
