#include <iostream>
#include "aws_cmd.h"

#define AWS_CMD "awscd"
#define AWS_CMD_USAGE "awscd <path>"

int main(int argc, char ** argv)
{
  return aws_cmd(argc, argv, AWS_CMD);
}
