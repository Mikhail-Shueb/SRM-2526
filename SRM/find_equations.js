const fs = require('fs');
const path = require('path');

const filePath = path.join(__dirname, 'Study_Guide.md');
const content = fs.readFileSync(filePath, 'utf8');
const lines = content.split('\n');

let inMathBlock = false;
let startLine = 0;
let currentBlock = [];

for (let idx = 0; idx < lines.length; idx++) {
  const line = lines[idx];
  if (line.trim() === '$$') {
    if (!inMathBlock) {
      inMathBlock = true;
      startLine = idx + 1;
      currentBlock = [];
    } else {
      inMathBlock = false;
      console.log(`Block from line ${startLine} to ${idx + 1}:`);
      console.log(currentBlock.join('\n'));
      console.log('---');
    }
  } else if (inMathBlock) {
    currentBlock.push(line);
  }
}
