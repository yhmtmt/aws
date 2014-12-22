#include <iostream>
#include "aws_cmd.h"

#define AWS_CMD "trat"
#define AWS_CMD_USAGE "trat <time rate in positive integer>"

int main(int argc, char ** argv)
{
  return aws_cmd(argc, argv, AWS_CMD);
}
