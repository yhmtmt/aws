#include <iostream>
#include "aws_cmd.h"


#define AWS_CMD "chrm"
#define AWS_CMD_USAGE "chrm <channel name>"

int main(int argc, char ** argv)
{
  return aws_cmd(argc, argv, AWS_CMD);
}
