

# **1) A) Solidity program to import a Contract into another Contract**

## **Answer**

### **Aim**

To write a Solidity program to import one contract into another contract.

### **Program**

**A.sol**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract A
{
    function showA() external pure returns(string memory)
    {
        return "Hello from A";
    }
}
```

**B.sol**

```solidity
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
```

### **Procedure**

1. Open Remix IDE
2. Create files `A.sol` and `B.sol`
3. Compile both contracts
4. Deploy `A.sol`
5. Copy deployed address of `A`
6. Deploy `B.sol` by passing address of `A`
7. Call `showB()`

### **Output**

```
Hello from A
```

---

# **1) B) Node JS program to demonstrate Digital Signature**

## **Answer**

### **Aim**

To create and verify a digital signature using Node.js.

### **Program**

```javascript
const prompt=require("prompt-sync")()
const { generateKeyPairSync, createSign, createVerify } = require("crypto");

const { publicKey, privateKey } = generateKeyPairSync("rsa", {
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
```

### **Procedure**

1. Create `DigitalSignature.js`
2. Run `npm install prompt-sync`
3. Execute `node DigitalSignature.js`

---

# **2) A) Solidity program to reverse a digit**

## **Answer**

### **Aim**

To write a Solidity program to reverse a given digit.

### **Program**

```solidity
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
```

### **Procedure**

1. Open Remix IDE
2. Compile and deploy contract
3. Call `getReverse()`

---

# **2) B) Transfer NEO funds between NEO accounts**

## **Answer**

### **Aim**

To transfer funds between NEO blockchain accounts.

### **Procedure**

1. Create wallets `genesis`, `mvsr`, `IOT`
2. Right-click `neo-express`
3. Select **Transfer assets**
4. Choose asset **NEO**
5. Transfer from `genesis` → `mvsr`
6. Transfer from `mvsr` → `IOT`

---

# **3) A) Solidity program to compute Student Grade Report using struct**

## **Answer**

### **Aim**

To compute student grade report using average marks with struct.

### **Program**

```solidity
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
```

---

# **3) B) Link Ganache in Rabby Wallet and perform Ether Transfer**

## **Answer**

### **Procedure**

1. Open Ganache GUI
2. Open Rabby Wallet
3. Add custom network
4. Import Ganache private key
5. Transfer ETH between accounts

---

# **4) A) Solidity program to display owner address and a message**

## **Answer**

```solidity
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
```

---

# **4) B) Link Ganache in Remix IDE and execute a smart contract**

## **Answer**

### **Procedure**

1. Open Ganache
2. Remix → Environment → Custom HTTP Provider
3. Enter `http://127.0.0.1:7545`
4. Deploy contract

---

# **5) A) Solidity Program to write Self Ether Transfer using mapping**

## **Answer**

```solidity
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
```

---

# **5) B) Transfer NEO funds between NEO accounts**

## **Answer**

Same procedure as **Question 2(B)** using NEO wallets.

---

# **6) A) Solidity program to perform Ether Transfer**

## **Answer**

```solidity
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
```

---

# **6) B) Node JS program to demonstrate Encryption and Decryption**

## **Answer**

```javascript
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
```

---

# **7) A) Solidity program to check whether a number is Gapful or not**

## **Answer**

```solidity
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
```

---

# **7) B) Link Ganache in Remix IDE and perform Ether Transfer**

## **Answer**

Use **Custom HTTP Provider (7545)** and deploy EtherTransfer contract to send Ether.

---

# **8) A) Solidity program to demonstrate Inheritance**

## **Answer**

```solidity
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
```

---

# **8) B) Node JS program to connect Ganache and display accounts**

## **Answer**

```javascript
const {Web3}=require("web3")
const web3=new Web3("http://127.0.0.1:7545")

async function run(){
 const acc=await web3.eth.getAccounts()
 for(let i=0;i<acc.length;i++)
  console.log(acc[i])
}
run()
```

---

# **9) A) Solidity program to read array dynamically and find sum of biggest and smallest**

## **Answer**

```solidity
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
```

---

# **9) B) Node JS program to display Ganache accounts and balances**

## **Answer**

Uses Web3 to fetch accounts and balances from Ganache.

---

# **10) A) Solidity program to demonstrate Parameterized Constructor**

## **Answer**

```solidity
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
```

---

# **10) B) Node JS program to transfer funds among dynamic accounts**

## **Answer**

Uses Web3 `sendTransaction()` with user-selected sender and receiver.

---

# **11) A) Bank DAPP using Metamask**

## **Answer**

```solidity
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
```

---

# **11) B) OpenSSL commands for encryption and decryption**

## **Answer**

```bash
openssl enc -aes256 -salt -in plaintext.txt -out encrypted.enc
openssl enc -d -aes256 -in encrypted.enc -out decrypted.txt
```

---

# **12) A) Bank DAPP using Ganache**

## **Answer**

Deploy Bank.sol using Ganache as local blockchain via Remix.

---

# **12) B) Creating and verifying Digital Signature using OpenSSL**

## **Answer**

```bash
openssl dgst -sha256 -sign private_key.pem -out file.sig demo.txt
openssl dgst -sha256 -verify public_key.pem -signature file.sig demo.txt
```

---

# **13) A) Solidity program to import a Contract into another Contract**

## **Answer**

Same program as **Question 1(A)**, showing contract import.

---

# **13) B) OpenSSL commands to generate public and private keys**

## **Answer**

```bash
openssl genpkey -algorithm RSA -out private_key.pem -aes256
openssl rsa -in private_key.pem -pubout -out public_key.pem
```

---

# **14) A) Solidity program to display owner address and message**

## **Answer**

Same Message contract program as Question 4(A).

---

# **14) B) Link Ganache in Remix IDE and execute a smart contract**

## **Answer**

Use Custom HTTP Provider and deploy smart contract via Remix.

---

# **15) A) Solidity program to demonstrate Inheritance**

## **Answer**

Same inheritance program as Question 8(A).

---

# **15) B) Link Ganache in Remix IDE and perform Ether Transfer**

## **Answer**

Deploy EtherTransfer contract and send Ether using Ganache network.


