#include <iostream>
#include "aws_cmd.h"


#define AWS_CMD "chinf"
#define AWS_CMD_USAGE "chinf [{<chan name> | n <chan id>}]"

int main(int argc, char ** argv)
{
  return aws_cmd(argc, argv, AWS_CMD);
}
