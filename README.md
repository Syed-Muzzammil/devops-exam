1)
A) Solidity program to import a Contract into another Contract
AIM

To write a Solidity program to import one contract into another contract.

PROGRAM

A.sol

// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract A
{
    function showA() external pure returns(string memory)
    {
        return "Hello from A";
    }
}


B.sol

// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

import "./A.sol";

contract B
{
    A a1;

    constructor(address _a1)
    {
        a1 = A(_a1);
    }

    function showB() public view returns(string memory)
    {
        return a1.showA();
    }
}

STEPS

Open Remix IDE

Create A.sol and B.sol

Compile both files

Deploy A.sol

Copy deployed address of A

Deploy B.sol and pass A’s address

Call showB()

OUTPUT
Hello from A

B) Node JS program to demonstrate Digital Signature
PROGRAM
const prompt=require("prompt-sync")()
const { generateKeyPairSync, createSign, createVerify } = require("crypto");

const { publicKey, privateKey } = generateKeyPairSync("rsa",
{
 modulusLength: 2048,
});

let msg = prompt("Enter a message:");

const sign = createSign("SHA256");
sign.update(msg);
sign.end();
const signature = sign.sign(privateKey, "hex");

console.log("Digital Signature created");

msg = prompt("Enter message to verify:");
const verify = createVerify("SHA256");
verify.update(msg);
verify.end();

console.log("Signature Verified:",
verify.verify(publicKey, signature, "hex"));

STEPS

Create file DigitalSignature.js

Run npm install prompt-sync

Execute node DigitalSignature.js

2)
A) Solidity program to reverse a digit
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract DigitReverse
{
    function getReverse(uint num) public pure returns (uint)
    {
        uint rev=0;
        uint rem=0;
        while (num!=0)
        {
            rem=num%10;
            rev=rev*10+rem;
            num/=10;
        }
        return rev;
    }
}

STEPS

Open Remix

Compile contract

Deploy

Call getReverse()

B) Transfer NEO funds between NEO accounts
STEPS

Create NEO wallets (genesis, mvsr, IOT)

Right click neo-express → Transfer assets

Select asset NEO

Transfer from genesis to mvsr

Transfer from mvsr to IOT

3)
A) Student Grade Report using struct
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract GradeReport
{
    struct student
    {
        string name;
        uint rollno;
        uint s1;
        uint s2;
        uint s3;
    }

    student[] public stds;

    function setStudent(string memory n,uint r,uint m1,uint m2,uint m3) public
    {
        stds.push(student(n,r,m1,m2,m3));
    }

    function getAverage(uint index) private view returns (uint)
    {
        uint total=stds[index].s1+stds[index].s2+stds[index].s3;
        return total/stds.length;
    }

    function getReport(uint index) public view returns (string memory)
    {
        uint report=getAverage(index);
        if (report>=70) return "Distinction";
        else if(report>=60) return "First class";
        else if(report>=50) return "Second class";
        else return "Fail";
    }
}

B) Link Ganache in Rabby Wallet and perform Ether Transfer
STEPS

Open Ganache GUI

Open Rabby Wallet

Add custom network

Import Ganache private key

Send ETH between accounts

4)
A) Display owner address and message
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract Message
{
    string message;
    address owner = msg.sender;

    function setMessage(string memory _m) public
    {
        message=_m;
    }

    function getMessage() public view returns(string memory)
    {
        return message;
    }

    function getOwner() public view returns(address)
    {
        return owner;
    }
}

B) Link Ganache in Remix IDE and execute smart contract
STEPS

Open Ganache

Remix → Environment → Custom HTTP Provider

URL: http://127.0.0.1:7545

Deploy contract

5)
A) Self Ether Transfer using mapping
contract EthersMapping
{
    mapping(address => uint) public balances;

    function deposit() public payable
    {
        balances[msg.sender]+=msg.value;
    }

    function getBalance() public view returns(uint)
    {
        return balances[msg.sender];
    }

    function withdraw(uint _amount) public
    {
        balances[msg.sender]-=_amount;
        payable(msg.sender).transfer(_amount);
    }
}

B) Transfer NEO funds between NEO accounts

✔ Same as 2-B

6)
A) Solidity program to perform Ether Transfer
contract EtherTransfer {
    address public sender;
    address public receiver;
    uint public amount;

    function sendEther(address payable _receiver) public payable {
        sender = msg.sender;
        receiver = _receiver;
        amount = msg.value;
        _receiver.transfer(msg.value);
    }
}

B) Node JS Encryption & Decryption
const crypto = require("crypto");
const prompt=require("prompt-sync")()

const algorithm="aes-256-cbc";
const key=crypto.randomBytes(32);
const iv=crypto.randomBytes(16);

const encrypt=(text)=>{
 let cipher=crypto.createCipheriv(algorithm,key,iv);
 return cipher.update(text,"utf8","hex")+cipher.final("hex");
};

const decrypt=(text)=>{
 let decipher=crypto.createDecipheriv(algorithm,key,iv);
 return decipher.update(text,"hex","utf8")+decipher.final("utf8");
};

let msg=prompt("Enter text:");
let enc=encrypt(msg);
console.log(decrypt(enc));

7)
A) Gapful Number
contract Gapful {
    function isGapful(uint num) private pure returns(bool){
        if(num<100) return false;
        uint last=num%10;
        uint first=num;
        while(first>=10) first/=10;
        uint div=first*10+last;
        return num%div==0;
    }

    function display(uint num) public pure returns(string memory){
        if(isGapful(num)) return "Gapful";
        else return "Not Gapful";
    }
}

B) Ether transfer using Ganache & Remix

✔ Same as 6-A with Ganache

8)
A) Inheritance
contract A {
    function showA() public pure returns(string memory){
        return "Hello from A contract";
    }
}

contract B is A {
    function showB() public pure returns(string memory){
        return "Hello from B Contract";
    }
}

B) Node JS – Display Ganache Accounts
const {Web3}=require("web3")
const web3=new Web3("http://127.0.0.1:7545")

async function run(){
 const acc=await web3.eth.getAccounts()
 for(let i=0;i<acc.length;i++)
  console.log(acc[i])
}
run()

9)
A) Dynamic Array – Max, Min
uint[] arr;

function readArray(uint v) public {
    arr.push(v);
}

function get() public view returns(uint,uint,uint){
    uint max=arr[0];
    uint min=arr[0];
    for(uint i=1;i<arr.length;i++){
        if(arr[i]>max) max=arr[i];
        if(arr[i]<min) min=arr[i];
    }
    return(max,min,max+min);
}

B) Node JS Ganache Accounts

✔ Same as 8-B

10)
A) Parameterized Constructor
contract ConstructorDemo {
    uint public x;
    uint public y;

    constructor(uint _x,uint _y){
        x=_x;
        y=_y;
    }

    function get() public view returns(uint){
        return x+y;
    }
}

B) Node JS Dynamic Fund Transfer

✔ From Ganache transfer program

11)
A) Bank DAPP using Metamask
contract Bank {
    uint public balance;
    address public owner;

    constructor(){
        balance=10000;
        owner=msg.sender;
    }

    function deposit(uint amt) public {
        balance+=amt;
    }

    function withdraw(uint amt) public {
        balance-=amt;
    }
}

B) OpenSSL Encryption & Decryption
openssl enc -aes256 -salt -in plaintext.txt -out encrypted.enc
openssl enc -d -aes256 -in encrypted.enc -out decrypted.txt

12)
A) Bank DAPP using Ganache

✔ Same Bank.sol + Ganache network

B) OpenSSL Digital Signature
openssl dgst -sha256 -sign private_key.pem -out file.sig demo.txt
openssl dgst -sha256 -verify public_key.pem -signature file.sig demo.txt

13)

✔ Same as 1-A

14)

✔ Same as 4-A & 4-B

15)

✔ Same as 8-A & 7-B
